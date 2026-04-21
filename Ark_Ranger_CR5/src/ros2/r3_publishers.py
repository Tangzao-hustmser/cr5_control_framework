"""ROS2 topic publishers for R3 runtime observability."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional
import json
import time

from r3_msgs.msg import CommandResult, RuntimeStatus, SafetyDecision, SystemEvent, ValidationIssue


class R3Ros2Publishers:
    def __init__(self, node: Any, runtime_cfg: Optional[Dict[str, Any]] = None) -> None:
        self._node = node
        self._runtime_cfg = runtime_cfg or {}
        ros2_cfg = self._runtime_cfg.get("ros2", {})
        topics = ros2_cfg.get("topics", {})
        qos_depth = int(ros2_cfg.get("qos_depth", 10))

        self._events_topic = str(topics.get("events", "/r3/events"))
        self._status_topic = str(topics.get("status", "/r3/status"))
        self._validation_topic = str(topics.get("validation", "/r3/validation"))

        self._events_pub = self._node.create_publisher(SystemEvent, self._events_topic, qos_depth)
        self._validation_pub = self._node.create_publisher(CommandResult, self._validation_topic, qos_depth)
        self._status_pub = self._node.create_publisher(RuntimeStatus, self._status_topic, qos_depth)

        self._status_interval_s = float(ros2_cfg.get("status_interval_s", 0.1))
        self._last_status_ts = 0.0

    @staticmethod
    def _safe_json(payload: Any) -> str:
        try:
            return json.dumps(payload, ensure_ascii=False)
        except Exception:
            return "{}"

    @staticmethod
    def _to_validation_issue(issue: Dict[str, Any]) -> ValidationIssue:
        msg = ValidationIssue()
        msg.fault_code = str(issue.get("fault_code", ""))
        msg.field = str(issue.get("field", ""))
        msg.message = str(issue.get("message", ""))
        return msg

    @staticmethod
    def _to_safety_msg(decision: str, reason_codes: Iterable[str], details: Dict[str, Any]) -> SafetyDecision:
        msg = SafetyDecision()
        msg.decision = str(decision)
        msg.reason_codes = [str(code) for code in reason_codes]
        msg.details_json = R3Ros2Publishers._safe_json(details)
        return msg

    def publish_event(self, event: Any, event_id: str = "", timeline_mapping: Optional[Dict[str, Any]] = None) -> None:
        msg = SystemEvent()
        timestamp_ms = int(getattr(event, "timestamp_ms", 0))
        command_id = str(getattr(event, "command_id", ""))
        resolved_event_id = str(event_id or f"evt-{timestamp_ms}-{command_id}")
        mapping = timeline_mapping if isinstance(timeline_mapping, dict) else {}
        msg.event_id = resolved_event_id
        msg.event_type = str(getattr(event, "event_type", ""))
        msg.command_id = command_id
        msg.timestamp_ms = timestamp_ms
        timeline_index = int(mapping.get("timeline_index", -1))
        if hasattr(msg, "timeline_index"):
            msg.timeline_index = timeline_index
        msg.fault_code = str(getattr(event, "fault_code", ""))
        msg.reason = str(getattr(event, "reason", ""))
        if hasattr(msg, "timeline_json"):
            msg.timeline_json = self._safe_json(mapping)
        raw_details = getattr(event, "details", {}) or {}
        details = dict(raw_details) if isinstance(raw_details, dict) else {}
        if isinstance(details, dict):
            details.setdefault("event_id", resolved_event_id)
            details.setdefault("timeline_index", timeline_index)
        msg.details_json = self._safe_json(details)
        self._events_pub.publish(msg)

    def publish_validation(
        self,
        command_id: str,
        ok: bool,
        issues: List[Dict[str, Any]],
        normalized: Optional[Dict[str, Any]],
    ) -> None:
        msg = CommandResult()
        msg.command_id = str(command_id)
        msg.timestamp_ms = int(time.time() * 1000)
        msg.ok = bool(ok)
        msg.status = "validation"
        msg.fault_code = "" if ok else str((issues[0].get("fault_code") if issues else ""))
        msg.message = "ok" if ok else "failed"
        msg.validation_issues = [self._to_validation_issue(issue) for issue in issues]
        msg.safety = self._to_safety_msg("", [], {})
        msg.details_json = self._safe_json({"normalized": normalized})
        self._validation_pub.publish(msg)

    def publish_safety(
        self,
        command_id: str,
        decision: str,
        reason_codes: Iterable[str],
        details: Dict[str, Any],
    ) -> None:
        msg = CommandResult()
        msg.command_id = str(command_id)
        msg.timestamp_ms = int(time.time() * 1000)
        msg.ok = decision != "block"
        msg.status = "safety"
        msg.fault_code = str(next(iter(reason_codes), "")) if decision == "block" else ""
        msg.message = "safety_block" if decision == "block" else decision
        msg.validation_issues = []
        msg.safety = self._to_safety_msg(decision, reason_codes, details)
        msg.details_json = self._safe_json(details)
        self._validation_pub.publish(msg)

    def publish_status(
        self,
        command_id: str,
        timestamp_ms: int,
        step_idx: int,
        state_payload: Dict[str, Any],
        ik_info: Optional[Dict[str, Any]] = None,
        gripper_info: Optional[Dict[str, Any]] = None,
        stage: str = "running",
    ) -> Optional[RuntimeStatus]:
        now = time.time()
        if self._status_interval_s > 0 and now - self._last_status_ts < self._status_interval_s:
            return None
        self._last_status_ts = now

        msg = RuntimeStatus()
        msg.command_id = str(command_id)
        msg.timestamp_ms = int(timestamp_ms)
        msg.step_idx = int(step_idx)
        msg.current_arm_rad = [float(v) for v in state_payload.get("current_arm_rad", [])]
        msg.target_arm_rad = [float(v) for v in state_payload.get("target_arm_rad", [])]
        msg.ee_pose_wxyz = [float(v) for v in state_payload.get("ee_pose_wxyz", [])]
        msg.gripper_position_rad = float(state_payload.get("gripper_position_rad", 0.0))
        msg.stage = str(stage)
        msg.info_json = self._safe_json({"ik": ik_info or {}, "gripper": gripper_info or {}})
        self._status_pub.publish(msg)
        return msg

    def make_safety_msg(self, decision: str, reason_codes: Iterable[str], details: Dict[str, Any]) -> SafetyDecision:
        return self._to_safety_msg(decision, reason_codes, details)
