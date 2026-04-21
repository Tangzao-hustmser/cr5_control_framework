import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import json
import threading

import numpy as np
import yaml

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from ark_patch import Node
from .bridge.ark_bridge import ArkIsaacBridge
from .r3.adapters import InputAdapter
from .r3.events import EVENT_TYPES, EventPublisher
from .r3.protocol import NormalizedCommand, generate_command_id
from .r3.safety import RobotStateSnapshot, SafetyFilterV1, SafetyDecision
from .r3.structured_logging import StructuredLogger
from .r3.timeline import TimelineIndexer
from .r3.validator import RuntimeModel, runtime_model_from_config, validate_and_normalize



@dataclass
class CommandHandleResult:
    command_id: str
    accepted: bool
    success: bool
    fault_code: str = ""
    message: str = ""
    validation_issues: List[Any] = field(default_factory=list)
    safety: Optional[SafetyDecision] = None
    normalized: Optional[NormalizedCommand] = None
    stage: str = "processed"
    status: Optional[Dict[str, Any]] = None

    def to_result_dict(self) -> Dict[str, Any]:
        normalized_json = ""
        if self.normalized is not None:
            normalized_json = json.dumps(self.normalized.to_dict(), ensure_ascii=False)
        safety_payload = None
        if self.safety is not None:
            safety_payload = {
                "decision": self.safety.decision,
                "reason_codes": list(self.safety.reason_codes),
                "details": dict(self.safety.details),
            }
        return {
            "command_id": self.command_id,
            "accepted": bool(self.accepted),
            "success": bool(self.success),
            "fault_code": self.fault_code or "",
            "message": self.message or "",
            "validation_issues": self.validation_issues,
            "safety": safety_payload,
            "normalized_json": normalized_json,
            "stage": self.stage,
            "status": self.status,
        }


class ArkIsaacSimNode(Node):
    def __init__(
        self,
        world: Any,
        config_path: str,
        runtime_cfg_path: str,
        adapters: Optional[List[InputAdapter]] = None,
        ros2_publisher: Optional[Any] = None,
        runtime_cfg_override: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__("isaac_bridge_node")
        self.world = world

        with open(config_path, "r", encoding="utf-8-sig") as fh:
            self.robot_cfg = yaml.safe_load(fh)
        if runtime_cfg_override is not None:
            self.runtime_cfg = runtime_cfg_override
        else:
            with open(runtime_cfg_path, "r", encoding="utf-8-sig") as fh:
                runtime_root = yaml.safe_load(fh)
            self.runtime_cfg = runtime_root.get("runtime", {})
        self.input_source = str(self.runtime_cfg.get("input_source", "ros2_action")).strip().lower()
        self.ros2_publisher = ros2_publisher
        self.config_path = config_path
        self.runtime_cfg_path = runtime_cfg_path
        self.project_root = root
        self._command_lock = threading.Lock()
        self._event_lock = threading.Lock()
        self._action_feedback_cb = None

        self.bridge = ArkIsaacBridge(config_path, self.world, runtime_cfg=self.runtime_cfg)
        self.runtime_model: RuntimeModel = runtime_model_from_config(self.robot_cfg)

        safety_cfg = self.runtime_cfg.get("safety", {})
        self.safety_filter = SafetyFilterV1(
            self.runtime_model,
            max_pose_deviation_m=float(safety_cfg.get("max_pose_deviation_m", 0.40)),
            max_joint_deviation_rad=float(safety_cfg.get("max_joint_deviation_rad", 1.20)),
        )

        log_root = self.runtime_cfg.get("logging", {}).get("root_dir", "reports/logs")
        log_root = os.path.join(root, log_root)
        self.logger = StructuredLogger(log_root)
        self.timeline = TimelineIndexer(self.logger.session_path)

        self.events = EventPublisher()
        self.events.subscribe(self._on_event)

        self.compat_adapters: List[InputAdapter] = adapters or []

        self.active_command: Optional[NormalizedCommand] = None
        self.last_observation = None
        self.last_status_snapshot: Optional[Dict[str, Any]] = None
        self._last_ik_info: Dict[str, Any] = {}
        self._last_gripper_info: Dict[str, Any] = {}
        self._last_safety_decision: Optional[SafetyDecision] = None
        self._event_seq = 0

        self.paused = False
        self.shutdown_requested = False
        status_cfg = self.runtime_cfg.get("status_output", {})
        self.status_output_enabled = bool(status_cfg.get("enabled", True))
        self.status_output_destination_cfg = str(status_cfg.get("destination", "auto")).strip().lower()
        self.status_output_interval_s = self._safe_float(status_cfg.get("interval_s", 1.0), default=1.0, min_value=0.1)
        self.status_output_pose_precision = self._safe_int(
            status_cfg.get("pose_precision", 3),
            default=3,
            min_value=1,
            max_value=8,
        )
        self.status_output_arm_precision = self._safe_int(
            status_cfg.get("arm_precision", 4),
            default=4,
            min_value=1,
            max_value=8,
        )
        self.status_output_grip_precision = self._safe_int(
            status_cfg.get("grip_precision", 4),
            default=4,
            min_value=1,
            max_value=8,
        )
        live_status_rel = str(status_cfg.get("live_status_file", "reports/live_status/current_status.json"))
        self.status_live_status_path = os.path.join(root, live_status_rel)
        self.status_output_destination = self._resolve_status_destination(self.status_output_destination_cfg)
        self._next_status_print_ts = 0.0

        if bool(self.runtime_cfg.get("compat", {}).get("enable_ark_subscription", False)):
            self.create_subscription(dict, "/robot/action", self._on_action_cb)

    def attach_ros2_publishers(self, publishers: Any) -> None:
        self.ros2_publisher = publishers

    def set_action_feedback_callback(self, callback: Optional[Any]) -> None:
        self._action_feedback_cb = callback

    @staticmethod
    def _safe_float(value: Any, default: float, min_value: Optional[float] = None) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = default
        if min_value is not None:
            parsed = max(parsed, min_value)
        return parsed

    @staticmethod
    def _safe_int(value: Any, default: int, min_value: Optional[int] = None, max_value: Optional[int] = None) -> int:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            parsed = default
        if min_value is not None:
            parsed = max(parsed, min_value)
        if max_value is not None:
            parsed = min(parsed, max_value)
        return parsed

    def _fmt(self, value: float, precision: int) -> str:
        return f"{float(value):.{precision}f}"

    def _format_arm(self, joint_values: List[float]) -> str:
        return "[" + ", ".join(self._fmt(v, self.status_output_arm_precision) for v in joint_values) + "]"

    def _resolve_status_destination(self, configured: str) -> str:
        valid_modes = {"auto", "console", "file", "both"}
        mode = configured if configured in valid_modes else "auto"
        if mode != "auto":
            return mode
        if self.input_source in ("local_terminal", "tcp_json"):
            return "file"
        return "console"

    def _status_to_console(self) -> bool:
        return self.status_output_destination in ("console", "both")

    def _status_to_file(self) -> bool:
        return self.status_output_destination in ("file", "both")

    def _write_live_status(self, payload: Dict[str, Any]) -> None:
        if not self._status_to_file():
            return
        try:
            parent = os.path.dirname(self.status_live_status_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            tmp_path = self.status_live_status_path + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self.status_live_status_path)
        except Exception:
            # Live status export should never break control loop.
            pass

    def _safe_print_status(self, message: str) -> None:
        try:
            print(message, flush=True)
        except OSError:
            # Some runners may close stdout/stderr before the sim loop exits.
            pass

    @property
    def log_session_path(self) -> str:
        return self.logger.session_path

    def initialize_physics(self) -> None:
        self.bridge.initialize_physics()
        for adapter in self.compat_adapters:
            adapter.start()

    def shutdown(self) -> None:
        for adapter in self.compat_adapters:
            try:
                adapter.stop()
            except Exception:
                pass

    def _on_event(self, event: Any) -> None:
        event_id = self._next_event_id(event.timestamp_ms, event.command_id)
        timeline_mapping = self.timeline.link_event(event_id, event.timestamp_ms, event.command_id, event.fault_code)
        event_payload = event.to_dict()
        event_payload["event_id"] = event_id
        event_payload["timeline"] = timeline_mapping
        self.logger.log_event(event.command_id, event_payload)
        if self.ros2_publisher is not None:
            try:
                self.ros2_publisher.publish_event(event, event_id=event_id, timeline_mapping=timeline_mapping)
            except Exception:
                pass

    def _next_event_id(self, timestamp_ms: int, command_id: str) -> str:
        with self._event_lock:
            self._event_seq += 1
            seq = self._event_seq
        return f"evt-{int(timestamp_ms)}-{seq:06d}-{command_id}"

    def _state_for_safety(self) -> RobotStateSnapshot:
        with self._command_lock:
            obs = self.last_observation
        if obs is None:
            return RobotStateSnapshot(
                joint_positions_rad=[0.0] * self.runtime_model.arm_joint_count,
                ee_pose_wxyz=[0.0] * 7,
                gripper_position_rad=0.0,
            )

        arm_joint_values = []
        for idx in self.bridge.arm_indices:
            if idx < len(obs.joint_positions):
                arm_joint_values.append(float(obs.joint_positions[idx]))
            else:
                arm_joint_values.append(0.0)

        return RobotStateSnapshot(
            joint_positions_rad=arm_joint_values,
            ee_pose_wxyz=[float(v) for v in obs.ee_pose.tolist()],
            gripper_position_rad=float(getattr(obs, "gripper_position", 0.0)),
        )

    def _on_action_cb(self, msg: Dict[str, Any]) -> None:
        self._process_raw(msg, source="ark_subscription")

    def _drain_compat_adapters(self) -> None:
        for adapter in self.compat_adapters:
            try:
                for message in adapter.poll():
                    self._process_raw(message.raw, message.source)
            except Exception as exc:
                self.events.publish(
                    EVENT_TYPES["COMMAND_REJECTED"],
                    command_id="adapter-error",
                    fault_code="R3-E1701",
                    reason="adapter_poll_failed",
                    details={"adapter": adapter.name, "error": str(exc)},
                )

    def _default_hold_command(self) -> NormalizedCommand:
        if self.last_observation is None:
            target = [0.0] * self.runtime_model.arm_joint_count
        else:
            target = []
            for idx in self.bridge.arm_indices:
                target.append(float(self.last_observation.joint_positions[idx]))

        return NormalizedCommand(
            command_id=generate_command_id(),
            protocol_version=self.runtime_cfg.get("protocol_version", "r3.v1"),
            timestamp_ms=int(time.time() * 1000),
            source="system",
            command_type="arm",
            arm_target_rad=target,
            metadata={"generated": "hold"},
        )

    def _handle_system_command(self, command: NormalizedCommand) -> None:
        op = command.system_operation
        with self._command_lock:
            if op == "pause":
                self.paused = True
            elif op == "resume":
                self.paused = False
            elif op == "reset":
                self.paused = False
                self.active_command = self._default_hold_command()
            elif op == "stop":
                self.shutdown_requested = True

        self.events.publish(
            EVENT_TYPES["SYSTEM_STATE_CHANGE"],
            command_id=command.command_id,
            fault_code="",
            reason=f"system_operation:{op}",
            details={"paused": self.paused, "shutdown_requested": self.shutdown_requested},
        )


    def _publish_validation(self, result: CommandHandleResult) -> None:
        if self.ros2_publisher is None:
            return
        try:
            normalized_dict = result.normalized.to_dict() if result.normalized else None
            self.ros2_publisher.publish_validation(
                result.command_id,
                bool(result.accepted and result.success),
                result.validation_issues,
                normalized_dict,
            )
        except Exception:
            pass

    def handle_command(self, raw: Any, source: Optional[str] = None) -> CommandHandleResult:
        src = source
        if not src:
            if isinstance(raw, dict):
                src = str(raw.get("source", "ros2_action"))
            else:
                src = "ros2_action"
        return self._process_raw(raw, src)

    def handle_system_operation(self, operation: str, reason: str = "", source: str = "ros2_service") -> CommandHandleResult:
        payload: Dict[str, Any] = {"operation": operation}
        if reason:
            payload["reason"] = reason
        raw = {"command_type": "system", "payload": payload}
        return self.handle_command(raw, source=source)

    def _process_raw(self, raw: Any, source: str) -> CommandHandleResult:
        if isinstance(raw, dict) and "__adapter_error__" in raw:
            err = raw["__adapter_error__"]
            cmd_id = generate_command_id()
            self.logger.log_command(cmd_id, source, raw)
            self.logger.log_validation(cmd_id, False, [err], None)
            self.events.publish(
                EVENT_TYPES["COMMAND_REJECTED"],
                command_id=cmd_id,
                fault_code=err.get("fault_code", "R3-E1001"),
                reason="adapter_parse_error",
                details={"source": source, "raw": err.get("raw_line", "")},
            )
            self._last_safety_decision = None
            result = CommandHandleResult(
                command_id=cmd_id,
                accepted=False,
                success=False,
                fault_code=err.get("fault_code", "R3-E1001"),
                message="adapter_parse_error",
                validation_issues=[err],
                stage="rejected",
                status=self.last_status_snapshot,
            )
            self._publish_validation(result)
            return result

        provisional_command_id = raw.get("command_id", generate_command_id()) if isinstance(raw, dict) else generate_command_id()
        normalized_raw = dict(raw) if isinstance(raw, dict) else raw
        if isinstance(normalized_raw, dict):
            normalized_raw.setdefault("command_id", provisional_command_id)

        self.logger.log_command(provisional_command_id, source, normalized_raw)
        validation = validate_and_normalize(normalized_raw, runtime=self.runtime_model, source=source)

        command_id = provisional_command_id
        if validation.command is not None:
            command_id = validation.command.command_id

        issue_dicts = [issue.to_dict() for issue in validation.issues]
        normalized_dict = validation.command.to_dict() if validation.command else None
        self.logger.log_validation(command_id, validation.ok, issue_dicts, normalized_dict)

        if not validation.ok or validation.command is None:
            first_fault = validation.first_fault() or "R3-E1999"
            self.events.publish(
                EVENT_TYPES["COMMAND_REJECTED"],
                command_id=command_id,
                fault_code=first_fault,
                reason="validation_failed",
                details={"issues": issue_dicts},
            )
            self._last_safety_decision = None
            result = CommandHandleResult(
                command_id=command_id,
                accepted=False,
                success=False,
                fault_code=first_fault,
                message="validation_failed",
                validation_issues=issue_dicts,
                stage="rejected",
                status=self.last_status_snapshot,
            )
            self._publish_validation(result)
            return result

        self.events.publish(
            EVENT_TYPES["COMMAND_NORMALIZED"],
            command_id=command_id,
            fault_code="",
            reason="validation_passed",
            details={"command_type": validation.command.command_type},
        )

        if validation.command.command_type == "system":
            self._handle_system_command(validation.command)
            self._last_safety_decision = None
            result = CommandHandleResult(
                command_id=command_id,
                accepted=True,
                success=True,
                message="system_executed",
                validation_issues=issue_dicts,
                normalized=validation.command,
                stage="system_executed",
                status=self.last_status_snapshot,
            )
            self._publish_validation(result)
            return result

        safety_state = self._state_for_safety()
        safety_decision = self.safety_filter.apply(validation.command, safety_state)

        self.logger.log_safety(
            command_id,
            safety_decision.decision,
            list(safety_decision.reason_codes),
            dict(safety_decision.details),
        )
        if self.ros2_publisher is not None:
            try:
                self.ros2_publisher.publish_safety(
                    command_id,
                    safety_decision.decision,
                    list(safety_decision.reason_codes),
                    dict(safety_decision.details),
                )
            except Exception:
                pass

        if safety_decision.decision == "block" or safety_decision.command is None:
            fault = safety_decision.reason_codes[0] if safety_decision.reason_codes else "R3-E1401"
            self.events.publish(
                EVENT_TYPES["SAFETY_BLOCK"],
                command_id=command_id,
                fault_code=fault,
                reason="safety_block",
                details=safety_decision.details,
            )
            self._last_safety_decision = safety_decision
            result = CommandHandleResult(
                command_id=command_id,
                accepted=False,
                success=False,
                fault_code=fault,
                message="safety_block",
                validation_issues=issue_dicts,
                safety=safety_decision,
                normalized=validation.command,
                stage="safety_block",
                status=self.last_status_snapshot,
            )
            self._publish_validation(result)
            return result

        stage = "accepted"
        fault_code = ""
        if safety_decision.decision == "clamp":
            self.events.publish(
                EVENT_TYPES["SAFETY_CLAMP"],
                command_id=command_id,
                fault_code="R3-E1402",
                reason="safety_clamp",
                details=safety_decision.details,
            )
            stage = "safety_clamp"
            fault_code = "R3-E1402"
        else:
            self.events.publish(
                EVENT_TYPES["SAFETY_PASS"],
                command_id=command_id,
                fault_code="",
                reason="safety_pass",
                details={},
            )

        self._last_safety_decision = safety_decision
        with self._command_lock:
            self.active_command = safety_decision.command
        result = CommandHandleResult(
            command_id=command_id,
            accepted=True,
            success=True,
            fault_code=fault_code,
            message="accepted",
            validation_issues=issue_dicts,
            safety=safety_decision,
            normalized=safety_decision.command,
            stage=stage,
            status=self.last_status_snapshot,
        )
        self._publish_validation(result)
        return result

    def step_node(self) -> None:
        if self.compat_adapters:
            self._drain_compat_adapters()

        with self._command_lock:
            if self.active_command is None:
                self.active_command = self._default_hold_command()
            command_to_run = self.active_command
            paused = self.paused

        if paused:
            command_to_run = self._default_hold_command()

        obs, execution_info = self.bridge.step(command_to_run)
        with self._command_lock:
            self.last_observation = obs

        command_id = command_to_run.command_id
        self.logger.log_ik(command_id, execution_info.get("ik", {}))

        state_payload = {
            "current_arm_rad": [float(obs.joint_positions[idx]) for idx in self.bridge.arm_indices],
            "target_arm_rad": execution_info.get("target_arm_rad", []),
            "ee_pose_wxyz": [float(v) for v in obs.ee_pose.tolist()],
            "gripper_position_rad": float(obs.gripper_position),
        }
        self.logger.log_state(command_id, state_payload)

        timestamp_ms = int(time.time() * 1000)
        step_idx = int(self.world.current_time_step_index)
        self.timeline.register_state_snapshot(f"state-{step_idx}", timestamp_ms, state_payload)
        self.timeline.register_video_frame(f"frame-{step_idx}", timestamp_ms, {"render_step": step_idx})

        ik_info = execution_info.get("ik", {})
        self._last_ik_info = dict(ik_info)
        if not ik_info.get("success", True):
            code = ik_info.get("reason_code", "R3-E1501")
            et = EVENT_TYPES["IK_FALLBACK"] if code == "R3-E1502" else EVENT_TYPES["IK_FAILURE"]
            self.events.publish(
                et,
                command_id=command_id,
                fault_code=code,
                reason="ik_failure",
                details=ik_info,
            )
        else:
            self.events.publish(
                EVENT_TYPES["IK_SUCCESS"],
                command_id=command_id,
                fault_code="",
                reason="ik_success",
                details=ik_info,
            )

        gripper_info = execution_info.get("gripper", {})
        self._last_gripper_info = dict(gripper_info)
        if gripper_info.get("stalled", False):
            self.events.publish(
                EVENT_TYPES["GRIPPER_STALL"],
                command_id=command_id,
                fault_code=gripper_info.get("reason_code", "R3-E1601"),
                reason="gripper_stall",
                details=gripper_info,
            )

        p = obs.ee_pose[:3]
        arm_state = state_payload["current_arm_rad"]
        status_snapshot = {
            "timestamp_ms": timestamp_ms,
            "step_idx": step_idx,
            "command_id": command_id,
            "ee_xyz": [float(p[0]), float(p[1]), float(p[2])],
            "arm_rad": [float(v) for v in arm_state],
            "gripper_rad": float(obs.gripper_position),
            "target_arm_rad": [float(v) for v in state_payload.get("target_arm_rad", [])],
            "mode": getattr(command_to_run, "command_type", ""),
        }
        self.last_status_snapshot = status_snapshot

        status_msg = None
        safety_msg = None
        if self.ros2_publisher is not None:
            try:
                status_msg = self.ros2_publisher.publish_status(
                    command_id=command_id,
                    timestamp_ms=timestamp_ms,
                    step_idx=step_idx,
                    state_payload=state_payload,
                    ik_info=ik_info,
                    gripper_info=gripper_info,
                    stage="running",
                )
                if self._last_safety_decision is not None:
                    safety_msg = self.ros2_publisher.make_safety_msg(
                        self._last_safety_decision.decision,
                        list(self._last_safety_decision.reason_codes),
                        dict(self._last_safety_decision.details),
                    )
            except Exception:
                pass
        if self._action_feedback_cb is not None and status_msg is not None:
            try:
                self._action_feedback_cb(status_msg, safety_msg, "running")
            except Exception:
                pass

        now = time.time()
        if now >= self._next_status_print_ts:
            self._next_status_print_ts = now + self.status_output_interval_s
            if self.status_output_enabled:
                self._write_live_status(status_snapshot)
                if self._status_to_console():
                    self._safe_print_status(
                        f"[Isaac][step={step_idx:06d}] cmd={command_id} "
                        f"EE xyz=({self._fmt(p[0], self.status_output_pose_precision)}, "
                        f"{self._fmt(p[1], self.status_output_pose_precision)}, "
                        f"{self._fmt(p[2], self.status_output_pose_precision)}) "
                        f"| arm={self._format_arm(arm_state)} "
                        f"| grip={self._fmt(obs.gripper_position, self.status_output_grip_precision)}"
                    )
