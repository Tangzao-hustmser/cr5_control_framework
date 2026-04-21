"""ROS2 Action server for R3 command execution."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional
import json
import threading
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from r3_msgs.action import Command
from r3_msgs.msg import CommandResult, RuntimeStatus, SafetyDecision, ValidationIssue

from src.r3.errors import get_fault_spec


@dataclass
class _GoalTracker:
    goal_handle: Any
    command_id: str
    done_event: threading.Event
    final_state: str = "succeeded"  # succeeded | aborted | canceled
    result_msg: Optional[CommandResult] = None


class R3ActionServer:
    def __init__(
        self,
        node: Any,
        isaac_node: Any,
        publishers: Optional[Any] = None,
        action_name: str = "/r3/command",
        action_cfg: Optional[Dict[str, Any]] = None,
    ) -> None:
        self._node = node
        self._isaac_node = isaac_node
        self._publishers = publishers
        self._action_name = action_name
        self._lock = threading.Lock()
        self._active: Optional[_GoalTracker] = None
        self._active_safety: Optional[SafetyDecision] = None

        cfg = action_cfg or {}
        self._result_timeout_s = float(cfg.get("result_timeout_s", 3.0))
        self._complete_on_accept = bool(cfg.get("complete_on_accept", False))

        cb_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self._node,
            Command,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # Hook feedback from Isaac loop to this action server.
        self._isaac_node.set_action_feedback_callback(self._publish_feedback)

    def shutdown(self) -> None:
        with self._lock:
            if self._active:
                self._active.final_state = "aborted"
                self._active.result_msg = self._build_result(
                    command_id=self._active.command_id,
                    ok=False,
                    status="shutdown",
                    fault_code="",
                    message="shutdown",
                )
                self._active.done_event.set()
                self._active = None
        try:
            self._server.destroy()
        except Exception:
            pass

    def _goal_callback(self, goal_request: Command.Goal) -> GoalResponse:
        if self._isaac_node.shutdown_requested:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> CancelResponse:
        with self._lock:
            if self._active and self._active.goal_handle == goal_handle:
                self._active.final_state = "canceled"
                self._active.result_msg = self._build_result(
                    command_id=self._active.command_id,
                    ok=False,
                    status="canceled",
                    fault_code="",
                    message="canceled",
                )
                self._active.done_event.set()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: Any) -> CommandResult:
        goal = goal_handle.request
        raw, parse_error = self._goal_to_raw(goal)
        if parse_error:
            result = self._build_result(
                command_id=parse_error.get("command_id", ""),
                ok=False,
                status="rejected",
                fault_code=parse_error.get("fault_code", "R3-E1001"),
                message=parse_error.get("message", "invalid_json"),
                validation_issues=parse_error.get("issues", []),
            )
            goal_handle.abort()
            return result

        handle_result = self._isaac_node.handle_command(raw, source="ros2_action")
        safety_msg = None
        if handle_result.safety is not None:
            safety_msg = self._to_safety_msg(
                {
                    "decision": handle_result.safety.decision,
                    "reason_codes": list(handle_result.safety.reason_codes),
                    "details": dict(handle_result.safety.details),
                }
            )

        if not (handle_result.accepted and handle_result.success):
            result = self._build_from_handle(handle_result, safety_msg)
            goal_handle.abort()
            return result

        command_type = ""
        if handle_result.normalized is not None:
            command_type = str(handle_result.normalized.command_type)

        # System commands complete synchronously.
        if command_type == "system":
            result = self._build_from_handle(handle_result, safety_msg)
            goal_handle.succeed()
            return result

        # Compat mode: close action immediately after command acceptance.
        if self._complete_on_accept:
            result = self._build_result(
                command_id=handle_result.command_id,
                ok=True,
                status="accepted",
                fault_code=handle_result.fault_code,
                message=handle_result.message or "accepted",
                safety_msg=safety_msg,
                details_json=json.dumps(handle_result.to_result_dict(), ensure_ascii=False),
            )
            goal_handle.succeed()
            return result

        tracker = _GoalTracker(goal_handle=goal_handle, command_id=handle_result.command_id, done_event=threading.Event())
        with self._lock:
            if self._active and self._active.command_id != tracker.command_id:
                self._active.final_state = "aborted"
                self._active.result_msg = self._build_result(
                    command_id=self._active.command_id,
                    ok=False,
                    status="superseded",
                    fault_code="",
                    message="superseded",
                )
                self._active.done_event.set()
            self._active = tracker
            self._active_safety = safety_msg

        done = tracker.done_event.wait(timeout=max(self._result_timeout_s, 0.1))
        with self._lock:
            if self._active is tracker:
                self._active = None
                self._active_safety = None

        if not done:
            result = self._build_result(
                command_id=tracker.command_id,
                ok=False,
                status="timeout_wait_feedback",
                fault_code="R3-E1999",
                message="timeout_wait_feedback",
            )
            goal_handle.abort()
            return result

        result = tracker.result_msg or self._build_result(
            command_id=tracker.command_id,
            ok=True,
            status="completed",
            fault_code="",
            message="completed",
        )

        if tracker.final_state == "succeeded":
            goal_handle.succeed()
        elif tracker.final_state == "canceled":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _goal_to_raw(self, goal: Command.Goal) -> tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
        cmd = goal.command
        command_id = str(cmd.command_id or "")
        timestamp_ms = int(cmd.timestamp_ms) if cmd.timestamp_ms else int(time.time() * 1000)
        payload_json = cmd.payload_json or "{}"
        metadata_json = cmd.metadata_json or "{}"

        try:
            payload = json.loads(payload_json) if payload_json else {}
        except Exception as exc:
            spec = get_fault_spec("R3-E1001")
            return {}, {
                "command_id": command_id,
                "fault_code": spec.code,
                "message": f"{spec.message} detail={str(exc)}",
                "issues": [{"fault_code": spec.code, "field": "payload_json", "message": str(exc)}],
            }
        if not isinstance(payload, dict):
            spec = get_fault_spec("R3-E1002")
            return {}, {
                "command_id": command_id,
                "fault_code": spec.code,
                "message": f"{spec.message} detail=payload_json is not object",
                "issues": [{"fault_code": spec.code, "field": "payload_json", "message": "not object"}],
            }

        try:
            metadata = json.loads(metadata_json) if metadata_json else {}
        except Exception as exc:
            spec = get_fault_spec("R3-E1001")
            return {}, {
                "command_id": command_id,
                "fault_code": spec.code,
                "message": f"{spec.message} detail={str(exc)}",
                "issues": [{"fault_code": spec.code, "field": "metadata_json", "message": str(exc)}],
            }
        if not isinstance(metadata, dict):
            spec = get_fault_spec("R3-E1002")
            return {}, {
                "command_id": command_id,
                "fault_code": spec.code,
                "message": f"{spec.message} detail=metadata_json is not object",
                "issues": [{"fault_code": spec.code, "field": "metadata_json", "message": "not object"}],
            }

        raw: Dict[str, Any] = {
            "protocol_version": str(cmd.protocol_version or "r3.v1"),
            "command_id": command_id or None,
            "timestamp_ms": timestamp_ms,
            "source": str(cmd.source or "ros2"),
            "command_type": str(cmd.command_type or ""),
            "payload": payload,
            "metadata": metadata,
        }
        return {k: v for k, v in raw.items() if v is not None}, None

    def _build_from_handle(self, handle_result: Any, safety_msg: Optional[SafetyDecision]) -> CommandResult:
        return self._build_result(
            command_id=handle_result.command_id,
            ok=bool(handle_result.accepted and handle_result.success),
            status=str(handle_result.stage),
            fault_code=handle_result.fault_code,
            message=handle_result.message,
            validation_issues=handle_result.validation_issues,
            safety_msg=safety_msg,
            details_json=json.dumps(handle_result.to_result_dict(), ensure_ascii=False),
        )

    def _build_result(
        self,
        command_id: str,
        ok: bool,
        status: str,
        fault_code: str,
        message: str,
        validation_issues: Optional[list[Dict[str, Any]]] = None,
        safety_msg: Optional[SafetyDecision] = None,
        details_json: Optional[str] = None,
    ) -> CommandResult:
        msg = CommandResult()
        msg.command_id = str(command_id)
        msg.timestamp_ms = int(time.time() * 1000)
        msg.ok = bool(ok)
        msg.status = str(status)
        msg.fault_code = str(fault_code)
        msg.message = str(message)
        msg.validation_issues = []
        if validation_issues:
            for issue in validation_issues:
                vi = ValidationIssue()
                vi.fault_code = str(issue.get("fault_code", ""))
                vi.field = str(issue.get("field", ""))
                vi.message = str(issue.get("message", ""))
                msg.validation_issues.append(vi)
        msg.safety = safety_msg if safety_msg is not None else SafetyDecision()
        msg.details_json = details_json or "{}"
        return msg

    def _to_safety_msg(self, decision: Dict[str, Any]) -> SafetyDecision:
        msg = SafetyDecision()
        msg.decision = str(decision.get("decision", ""))
        msg.reason_codes = [str(code) for code in decision.get("reason_codes", [])]
        msg.details_json = json.dumps(decision.get("details", {}), ensure_ascii=False)
        return msg

    @staticmethod
    def _status_msg_to_dict(status_msg: RuntimeStatus) -> Dict[str, Any]:
        return {
            "command_id": str(status_msg.command_id),
            "timestamp_ms": int(status_msg.timestamp_ms),
            "step_idx": int(status_msg.step_idx),
            "current_arm_rad": [float(v) for v in status_msg.current_arm_rad],
            "target_arm_rad": [float(v) for v in status_msg.target_arm_rad],
            "ee_pose_wxyz": [float(v) for v in status_msg.ee_pose_wxyz],
            "gripper_position_rad": float(status_msg.gripper_position_rad),
            "stage": str(status_msg.stage),
            "info_json": str(status_msg.info_json),
        }

    def _publish_feedback(self, status_msg: Any, safety_msg: Optional[SafetyDecision], stage: str) -> None:
        with self._lock:
            tracker = self._active
            active_safety = self._active_safety
        if tracker is None:
            return
        if status_msg.command_id != tracker.command_id:
            return

        feedback = Command.Feedback()
        feedback.status = status_msg
        feedback.safety = safety_msg if safety_msg is not None else SafetyDecision()
        feedback.stage = str(stage)
        try:
            tracker.goal_handle.publish_feedback(feedback)
        except Exception:
            pass

        if tracker.done_event.is_set():
            return

        resolved_safety = safety_msg if safety_msg is not None else active_safety
        details = {
            "feedback_stage": str(stage),
            "status": self._status_msg_to_dict(status_msg),
        }
        if resolved_safety is not None:
            details["safety"] = {
                "decision": str(resolved_safety.decision),
                "reason_codes": [str(v) for v in resolved_safety.reason_codes],
                "details_json": str(resolved_safety.details_json),
            }
        tracker.result_msg = self._build_result(
            command_id=tracker.command_id,
            ok=True,
            status="completed",
            fault_code="",
            message="completed",
            safety_msg=resolved_safety,
            details_json=json.dumps(details, ensure_ascii=False),
        )
        tracker.final_state = "succeeded"
        tracker.done_event.set()
