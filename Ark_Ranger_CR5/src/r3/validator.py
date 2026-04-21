"""Command validator + normalizer for unified R3 protocol."""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import math

from .errors import get_fault_spec
from .protocol import (
    COMMAND_TYPES,
    GRIPPER_MODES,
    PAYLOAD_FIELDS,
    PROTOCOL_VERSION,
    SYSTEM_OPERATIONS,
    TOP_LEVEL_FIELDS,
    NormalizedCommand,
    ValidationIssue,
    ValidationResult,
    normalize_base_envelope,
)


@dataclass
class RuntimeModel:
    arm_joint_names: List[str]
    arm_joint_limits_rad: List[Tuple[float, float]]
    gripper_joint_name: str
    gripper_limits_rad: Tuple[float, float]
    allowed_frames: List[str] = field(default_factory=lambda: ["world"])
    workspace_limits_m: Dict[str, Tuple[float, float]] = field(
        default_factory=lambda: {
            "x": (-1.2, 1.2),
            "y": (-1.2, 1.2),
            "z": (0.0, 1.8),
        }
    )
    joint_reject_margin_rad: float = 1.2
    position_reject_margin_m: float = 0.5

    @property
    def arm_joint_count(self) -> int:
        return len(self.arm_joint_names)



def _issue(code: str, field: str, message: str) -> ValidationIssue:
    spec = get_fault_spec(code)
    return ValidationIssue(fault_code=spec.code, field=field, message=message)



def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))



def _validate_number_list(value: Any, expected_len: int, field: str, issues: List[ValidationIssue]) -> Optional[List[float]]:
    if not isinstance(value, list):
        issues.append(_issue("R3-E1103", field, f"{field} must be a list."))
        return None
    if len(value) != expected_len:
        issues.append(_issue("R3-E1201", field, f"{field} length must be {expected_len}."))
        return None
    out: List[float] = []
    for idx, item in enumerate(value):
        if not _is_number(item):
            issues.append(_issue("R3-E1103", f"{field}[{idx}]", "Value must be finite number."))
            return None
        out.append(float(item))
    return out



def runtime_model_from_config(robot_cfg: Dict[str, Any]) -> RuntimeModel:
    arm_cfg = robot_cfg.get("action_space", {}).get("arm", {})
    gripper_cfg = robot_cfg.get("action_space", {}).get("gripper", {})

    arm_joint_names = list(arm_cfg.get("joints", []))
    limits_cfg = arm_cfg.get("joint_limits", {})
    limits: List[Tuple[float, float]] = []
    default_limit = tuple(arm_cfg.get("default_limit", [-3.14, 3.14]))
    for name in arm_joint_names:
        v = limits_cfg.get(name, default_limit)
        limits.append((float(v[0]), float(v[1])))

    gripper_joint_names = list(gripper_cfg.get("joints", ["finger_joint"]))
    g_limits = gripper_cfg.get("joint_limits", {}).get(gripper_joint_names[0], [0.0, 0.6524])

    workspace = robot_cfg.get("action_space", {}).get("pose", {}).get(
        "workspace_m",
        {
            "x": [-1.2, 1.2],
            "y": [-1.2, 1.2],
            "z": [0.0, 1.8],
        },
    )

    frames = robot_cfg.get("coordinate_frames", {}).get("allowed", ["world"])

    return RuntimeModel(
        arm_joint_names=arm_joint_names,
        arm_joint_limits_rad=limits,
        gripper_joint_name=gripper_joint_names[0],
        gripper_limits_rad=(float(g_limits[0]), float(g_limits[1])),
        allowed_frames=[str(f) for f in frames],
        workspace_limits_m={
            "x": (float(workspace["x"][0]), float(workspace["x"][1])),
            "y": (float(workspace["y"][0]), float(workspace["y"][1])),
            "z": (float(workspace["z"][0]), float(workspace["z"][1])),
        },
    )



def validate_and_normalize(raw_message: Any, runtime: RuntimeModel, source: str) -> ValidationResult:
    issues: List[ValidationIssue] = []

    if not isinstance(raw_message, dict):
        issues.append(_issue("R3-E1002", "message", "Input message must be an object."))
        return ValidationResult(ok=False, command=None, issues=issues)

    envelope = normalize_base_envelope(raw_message, source=source)

    unknown_top = set(envelope.keys()) - TOP_LEVEL_FIELDS
    for field in sorted(unknown_top):
        issues.append(_issue("R3-E1102", field, f"Unknown top-level field: {field}"))

    for required in ("protocol_version", "command_id", "timestamp_ms", "source", "command_type", "payload"):
        if required not in envelope:
            issues.append(_issue("R3-E1101", required, f"Missing required field: {required}"))

    if issues:
        return ValidationResult(ok=False, command=None, issues=issues)

    if envelope["protocol_version"] != PROTOCOL_VERSION:
        issues.append(_issue("R3-E1003", "protocol_version", "Unsupported protocol version."))

    if not isinstance(envelope["command_id"], str) or not envelope["command_id"].strip():
        issues.append(_issue("R3-E1103", "command_id", "command_id must be non-empty string."))

    if not _is_number(envelope["timestamp_ms"]):
        issues.append(_issue("R3-E1103", "timestamp_ms", "timestamp_ms must be number."))

    if not isinstance(envelope["source"], str):
        issues.append(_issue("R3-E1103", "source", "source must be string."))

    command_type = envelope["command_type"]
    if command_type not in COMMAND_TYPES:
        issues.append(_issue("R3-E1301", "command_type", f"Unsupported command_type: {command_type}"))

    payload = envelope["payload"]
    if not isinstance(payload, dict):
        issues.append(_issue("R3-E1103", "payload", "payload must be object."))

    if issues:
        return ValidationResult(ok=False, command=None, issues=issues)

    unknown_payload = set(payload.keys()) - PAYLOAD_FIELDS[command_type]
    for field in sorted(unknown_payload):
        issues.append(_issue("R3-E1102", f"payload.{field}", f"Unknown payload field: {field}"))

    pose_target: Optional[List[float]] = None
    arm_target: Optional[List[float]] = None
    gripper_mode: Optional[str] = None
    gripper_position: Optional[float] = None
    system_operation: Optional[str] = None
    pose_frame = "world"

    if command_type == "pose":
        for required in ("position_m", "orientation_wxyz"):
            if required not in payload:
                issues.append(_issue("R3-E1101", f"payload.{required}", f"Missing required field: {required}"))

        pos = _validate_number_list(payload.get("position_m"), 3, "payload.position_m", issues)
        quat = _validate_number_list(payload.get("orientation_wxyz"), 4, "payload.orientation_wxyz", issues)
        pose_frame = str(payload.get("frame", "world"))

        if pose_frame not in runtime.allowed_frames:
            issues.append(_issue("R3-E1204", "payload.frame", f"Unsupported frame: {pose_frame}"))

        if pos is not None:
            ranges = runtime.workspace_limits_m
            for axis_name, value in zip(("x", "y", "z"), pos):
                lo, hi = ranges[axis_name]
                if value < lo - runtime.position_reject_margin_m or value > hi + runtime.position_reject_margin_m:
                    issues.append(
                        _issue(
                            "R3-E1202",
                            f"payload.position_m.{axis_name}",
                            f"{axis_name} is far outside allowed workspace.",
                        )
                    )

        if quat is not None:
            norm = math.sqrt(sum(v * v for v in quat))
            if norm < 1e-8:
                issues.append(_issue("R3-E1202", "payload.orientation_wxyz", "Quaternion norm must be > 0."))
            else:
                quat = [v / norm for v in quat]

        if pos is not None and quat is not None:
            pose_target = [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]

    elif command_type == "arm":
        if "joint_positions_rad" not in payload:
            issues.append(_issue("R3-E1101", "payload.joint_positions_rad", "Missing joint_positions_rad."))

        positions = _validate_number_list(
            payload.get("joint_positions_rad"),
            runtime.arm_joint_count,
            "payload.joint_positions_rad",
            issues,
        )

        payload_joint_names = payload.get("joint_names")
        if payload_joint_names is not None:
            if not isinstance(payload_joint_names, list) or len(payload_joint_names) != runtime.arm_joint_count:
                issues.append(_issue("R3-E1201", "payload.joint_names", "joint_names length mismatch."))
            elif [str(v) for v in payload_joint_names] != runtime.arm_joint_names:
                issues.append(
                    _issue(
                        "R3-E1202",
                        "payload.joint_names",
                        "joint_names must match configured arm joint order.",
                    )
                )

        if positions is not None:
            for idx, (value, limits) in enumerate(zip(positions, runtime.arm_joint_limits_rad)):
                lo, hi = limits
                if value < lo - runtime.joint_reject_margin_rad or value > hi + runtime.joint_reject_margin_rad:
                    issues.append(
                        _issue(
                            "R3-E1202",
                            f"payload.joint_positions_rad[{idx}]",
                            f"Joint {runtime.arm_joint_names[idx]} is far outside allowed range.",
                        )
                    )
            arm_target = positions

    elif command_type == "gripper":
        if "mode" not in payload:
            issues.append(_issue("R3-E1101", "payload.mode", "Missing gripper mode."))
        mode = str(payload.get("mode", ""))
        if mode not in GRIPPER_MODES:
            issues.append(_issue("R3-E1103", "payload.mode", f"Unsupported gripper mode: {mode}"))
        gripper_mode = mode

        if mode == "position":
            if "position_rad" not in payload:
                issues.append(_issue("R3-E1101", "payload.position_rad", "position mode requires position_rad."))
            elif not _is_number(payload.get("position_rad")):
                issues.append(_issue("R3-E1103", "payload.position_rad", "position_rad must be number."))
            else:
                gripper_position = float(payload["position_rad"])
                lo, hi = runtime.gripper_limits_rad
                margin = 0.2
                if gripper_position < lo - margin or gripper_position > hi + margin:
                    issues.append(_issue("R3-E1202", "payload.position_rad", "position_rad is far outside gripper range."))

    elif command_type == "system":
        if "operation" not in payload:
            issues.append(_issue("R3-E1101", "payload.operation", "Missing system operation."))
        operation = str(payload.get("operation", ""))
        if operation not in SYSTEM_OPERATIONS:
            issues.append(_issue("R3-E1103", "payload.operation", f"Unsupported operation: {operation}"))
        system_operation = operation

    if issues:
        return ValidationResult(ok=False, command=None, issues=issues)

    cmd = NormalizedCommand(
        command_id=str(envelope["command_id"]),
        protocol_version=str(envelope["protocol_version"]),
        timestamp_ms=int(float(envelope["timestamp_ms"])),
        source=str(envelope["source"]),
        command_type=command_type,
        pose_target_wxyz=pose_target,
        pose_frame=pose_frame,
        arm_target_rad=arm_target,
        gripper_mode=gripper_mode,
        gripper_position_rad=gripper_position,
        system_operation=system_operation,
        metadata=dict(envelope.get("metadata", {})),
        raw_message=dict(raw_message),
    )

    return ValidationResult(ok=True, command=cmd, issues=[])
