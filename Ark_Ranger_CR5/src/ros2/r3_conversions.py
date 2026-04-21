"""Backward-compatible conversion helpers for ROS2 <-> r3.v1 dictionaries."""

from __future__ import annotations

import json
import time
from typing import Any, Dict, List, Optional

from src.ros2.r3_msg_utils import command_dict_to_msg as _command_dict_to_msg
from src.ros2.r3_msg_utils import command_msg_to_dict as _command_msg_to_dict


def command_msg_to_dict(msg: Any, default_source: str = "ros2") -> Dict[str, Any]:
    return _command_msg_to_dict(msg, default_source=default_source)


def command_dict_to_msg(command: Dict[str, Any]) -> Any:
    return _command_dict_to_msg(command)


def validation_issues_to_msgs(issues: List[Any]) -> List[Any]:
    from r3_msgs.msg import ValidationIssue

    out: List[Any] = []
    for issue in issues:
        if hasattr(issue, "to_dict"):
            data = issue.to_dict()
        elif isinstance(issue, dict):
            data = issue
        else:
            data = {
                "fault_code": str(getattr(issue, "fault_code", "")),
                "field": str(getattr(issue, "field", "")),
                "message": str(getattr(issue, "message", "")),
            }
        msg = ValidationIssue()
        msg.fault_code = str(data.get("fault_code", ""))
        msg.field = str(data.get("field", ""))
        msg.message = str(data.get("message", ""))
        out.append(msg)
    return out


def safety_decision_to_msg(decision: Optional[Any]) -> Any:
    from r3_msgs.msg import SafetyDecision

    msg = SafetyDecision()
    if decision is None:
        msg.decision = ""
        msg.reason_codes = []
        msg.details_json = ""
        return msg

    if isinstance(decision, dict):
        msg.decision = str(decision.get("decision", ""))
        msg.reason_codes = [str(v) for v in decision.get("reason_codes", [])]
        msg.details_json = json.dumps(decision.get("details", {}), ensure_ascii=False)
        return msg

    msg.decision = str(getattr(decision, "decision", ""))
    msg.reason_codes = [str(v) for v in getattr(decision, "reason_codes", [])]
    details = getattr(decision, "details", {})
    msg.details_json = json.dumps(details if isinstance(details, dict) else {}, ensure_ascii=False)
    return msg


def runtime_status_to_msg(status: Dict[str, Any]) -> Any:
    from r3_msgs.msg import RuntimeStatus

    msg = RuntimeStatus()
    msg.command_id = str(status.get("command_id", ""))
    msg.timestamp_ms = int(status.get("timestamp_ms", 0) or 0)
    msg.step_idx = int(status.get("step_idx", 0) or 0)
    msg.current_arm_rad = [float(v) for v in status.get("current_arm_rad", [])]
    msg.target_arm_rad = [float(v) for v in status.get("target_arm_rad", [])]
    msg.ee_pose_wxyz = [float(v) for v in status.get("ee_pose_wxyz", [])]
    msg.gripper_position_rad = float(status.get("gripper_position_rad", 0.0) or 0.0)
    msg.stage = str(status.get("stage", ""))
    msg.info_json = str(status.get("info_json", ""))
    return msg


def command_result_to_msg(result: Dict[str, Any]) -> Any:
    from r3_msgs.msg import CommandResult

    accepted = bool(result.get("accepted", False))
    success = bool(result.get("success", False))
    msg = CommandResult()
    msg.command_id = str(result.get("command_id", ""))
    msg.timestamp_ms = int(result.get("timestamp_ms", int(time.time() * 1000)))
    msg.ok = bool(accepted and success)
    msg.status = str(result.get("stage", "processed"))
    msg.fault_code = str(result.get("fault_code", ""))
    msg.message = str(result.get("message", ""))
    msg.validation_issues = validation_issues_to_msgs(result.get("validation_issues", []))
    msg.safety = safety_decision_to_msg(result.get("safety"))
    msg.details_json = json.dumps(result, ensure_ascii=False)
    return msg
