import os
import sys
import time
from typing import Any, Dict, List, Optional
import json

import numpy as np
import yaml

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from ark_patch import Node
from .bridge.ark_bridge import ArkIsaacBridge
from .r3.adapters import AdapterMessage, InputAdapter
from .r3.events import EVENT_TYPES, EventPublisher
from .r3.protocol import NormalizedCommand, generate_command_id
from .r3.safety import RobotStateSnapshot, SafetyFilterV1
from .r3.structured_logging import StructuredLogger
from .r3.timeline import TimelineIndexer
from .r3.validator import RuntimeModel, runtime_model_from_config, validate_and_normalize


class ArkIsaacSimNode(Node):
    def __init__(
        self,
        world: Any,
        config_path: str,
        runtime_cfg_path: str,
        adapters: Optional[List[InputAdapter]] = None,
    ) -> None:
        super().__init__("isaac_bridge_node")
        self.world = world

        with open(config_path, "r", encoding="utf-8-sig") as fh:
            self.robot_cfg = yaml.safe_load(fh)
        with open(runtime_cfg_path, "r", encoding="utf-8-sig") as fh:
            runtime_root = yaml.safe_load(fh)
        self.runtime_cfg = runtime_root.get("runtime", {})

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

        self.adapters: List[InputAdapter] = adapters or []
        self.pending_messages: List[AdapterMessage] = []

        self.active_command: Optional[NormalizedCommand] = None
        self.last_observation = None

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

        # Keep existing subscription entry but enforce unified protocol.
        self.create_subscription(dict, "/robot/action", self._on_action_cb)

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
        has_local_terminal = any(getattr(adapter, "name", "") == "local_terminal" for adapter in self.adapters)
        return "file" if has_local_terminal else "console"

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
        for adapter in self.adapters:
            adapter.start()

    def shutdown(self) -> None:
        for adapter in self.adapters:
            try:
                adapter.stop()
            except Exception:
                pass

    def _on_event(self, event: Any) -> None:
        self.logger.log_event(event.command_id, event.to_dict())
        event_id = f"evt-{event.timestamp_ms}-{event.command_id}"
        self.timeline.link_event(event_id, event.timestamp_ms, event.command_id, event.fault_code)

    def _state_for_safety(self) -> RobotStateSnapshot:
        if self.last_observation is None:
            return RobotStateSnapshot(
                joint_positions_rad=[0.0] * self.runtime_model.arm_joint_count,
                ee_pose_wxyz=[0.0] * 7,
                gripper_position_rad=0.0,
            )

        obs = self.last_observation
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
        self.pending_messages.append(
            AdapterMessage(source="ark_subscription", raw=msg, received_ms=int(time.time() * 1000))
        )

    def _poll_adapters(self) -> None:
        for adapter in self.adapters:
            try:
                self.pending_messages.extend(adapter.poll())
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

    def _process_message(self, message: AdapterMessage) -> None:
        raw = message.raw
        if isinstance(raw, dict) and "__adapter_error__" in raw:
            err = raw["__adapter_error__"]
            cmd_id = generate_command_id()
            self.logger.log_command(cmd_id, message.source, raw)
            self.logger.log_validation(cmd_id, False, [err], None)
            self.events.publish(
                EVENT_TYPES["COMMAND_REJECTED"],
                command_id=cmd_id,
                fault_code=err.get("fault_code", "R3-E1001"),
                reason="adapter_parse_error",
                details={"source": message.source, "raw": err.get("raw_line", "")},
            )
            return

        provisional_command_id = raw.get("command_id", generate_command_id()) if isinstance(raw, dict) else generate_command_id()
        normalized_raw = dict(raw) if isinstance(raw, dict) else raw
        if isinstance(normalized_raw, dict):
            normalized_raw.setdefault("command_id", provisional_command_id)

        self.logger.log_command(provisional_command_id, message.source, normalized_raw)
        validation = validate_and_normalize(normalized_raw, runtime=self.runtime_model, source=message.source)

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
            return

        self.events.publish(
            EVENT_TYPES["COMMAND_NORMALIZED"],
            command_id=command_id,
            fault_code="",
            reason="validation_passed",
            details={"command_type": validation.command.command_type},
        )

        if validation.command.command_type == "system":
            self._handle_system_command(validation.command)
            return

        safety_state = self._state_for_safety()
        safety_decision = self.safety_filter.apply(validation.command, safety_state)

        self.logger.log_safety(
            command_id,
            safety_decision.decision,
            list(safety_decision.reason_codes),
            dict(safety_decision.details),
        )

        if safety_decision.decision == "block" or safety_decision.command is None:
            fault = safety_decision.reason_codes[0] if safety_decision.reason_codes else "R3-E1401"
            self.events.publish(
                EVENT_TYPES["SAFETY_BLOCK"],
                command_id=command_id,
                fault_code=fault,
                reason="safety_block",
                details=safety_decision.details,
            )
            return

        if safety_decision.decision == "clamp":
            self.events.publish(
                EVENT_TYPES["SAFETY_CLAMP"],
                command_id=command_id,
                fault_code="R3-E1402",
                reason="safety_clamp",
                details=safety_decision.details,
            )
        else:
            self.events.publish(
                EVENT_TYPES["SAFETY_PASS"],
                command_id=command_id,
                fault_code="",
                reason="safety_pass",
                details={},
            )

        self.active_command = safety_decision.command

    def step_node(self) -> None:
        self._poll_adapters()

        while self.pending_messages:
            msg = self.pending_messages.pop(0)
            self._process_message(msg)

        if self.active_command is None:
            self.active_command = self._default_hold_command()

        command_to_run = self.active_command
        if self.paused:
            command_to_run = self._default_hold_command()

        obs, execution_info = self.bridge.step(command_to_run)
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
        if gripper_info.get("stalled", False):
            self.events.publish(
                EVENT_TYPES["GRIPPER_STALL"],
                command_id=command_id,
                fault_code=gripper_info.get("reason_code", "R3-E1601"),
                reason="gripper_stall",
                details=gripper_info,
            )

        now = time.time()
        if self.status_output_enabled and now >= self._next_status_print_ts:
            self._next_status_print_ts = now + self.status_output_interval_s
            p = obs.ee_pose[:3]
            arm_state = state_payload["current_arm_rad"]
            status_snapshot = {
                "timestamp_ms": timestamp_ms,
                "step_idx": step_idx,
                "command_id": command_id,
                "ee_xyz": [float(p[0]), float(p[1]), float(p[2])],
                "arm_rad": [float(v) for v in arm_state],
                "gripper_rad": float(obs.gripper_position),
            }
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
