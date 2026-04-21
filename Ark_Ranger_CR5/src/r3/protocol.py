"""Unified action protocol model for Ark Ranger CR5."""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import uuid
import time


PROTOCOL_VERSION = "r3.v1"
COMMAND_TYPES = ("pose", "arm", "gripper", "system")
GRIPPER_MODES = ("open", "close", "hold", "position")
SYSTEM_OPERATIONS = ("pause", "resume", "reset", "stop")

TOP_LEVEL_FIELDS = {
    "protocol_version",
    "command_id",
    "timestamp_ms",
    "source",
    "command_type",
    "payload",
    "metadata",
}

PAYLOAD_FIELDS = {
    "pose": {"frame", "position_m", "orientation_wxyz"},
    "arm": {"joint_positions_rad", "joint_names"},
    "gripper": {"mode", "position_rad", "force_ratio", "speed_ratio"},
    "system": {"operation", "reason"},
}


@dataclass
class NormalizedCommand:
    command_id: str
    protocol_version: str
    timestamp_ms: int
    source: str
    command_type: str
    pose_target_wxyz: Optional[List[float]] = None
    pose_frame: str = "world"
    arm_target_rad: Optional[List[float]] = None
    gripper_mode: Optional[str] = None
    gripper_position_rad: Optional[float] = None
    system_operation: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    raw_message: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "command_id": self.command_id,
            "protocol_version": self.protocol_version,
            "timestamp_ms": self.timestamp_ms,
            "source": self.source,
            "command_type": self.command_type,
            "pose_target_wxyz": self.pose_target_wxyz,
            "pose_frame": self.pose_frame,
            "arm_target_rad": self.arm_target_rad,
            "gripper_mode": self.gripper_mode,
            "gripper_position_rad": self.gripper_position_rad,
            "system_operation": self.system_operation,
            "metadata": dict(self.metadata),
        }


@dataclass
class ValidationIssue:
    fault_code: str
    field: str
    message: str

    def to_dict(self) -> Dict[str, str]:
        return {
            "fault_code": self.fault_code,
            "field": self.field,
            "message": self.message,
        }


@dataclass
class ValidationResult:
    ok: bool
    command: Optional[NormalizedCommand]
    issues: List[ValidationIssue]

    def first_fault(self) -> str:
        if not self.issues:
            return ""
        return self.issues[0].fault_code



def generate_command_id() -> str:
    return f"cmd-{uuid.uuid4().hex[:16]}"


def normalize_base_envelope(raw: Dict[str, Any], source: str) -> Dict[str, Any]:
    """Fill defaults for envelope-level fields before strict validation."""
    normalized = dict(raw)
    normalized.setdefault("protocol_version", PROTOCOL_VERSION)
    normalized.setdefault("command_id", generate_command_id())
    normalized.setdefault("timestamp_ms", int(time.time() * 1000))
    normalized.setdefault("source", source)
    normalized.setdefault("metadata", {})
    return normalized


def protocol_example_messages() -> Dict[str, Dict[str, Any]]:
    return {
        "pose": {
            "protocol_version": PROTOCOL_VERSION,
            "command_id": "cmd-001",
            "timestamp_ms": 1710000000000,
            "source": "terminal",
            "command_type": "pose",
            "payload": {
                "frame": "world",
                "position_m": [0.45, 0.10, 0.55],
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "metadata": {"operator": "demo"},
        },
        "arm": {
            "protocol_version": PROTOCOL_VERSION,
            "command_id": "cmd-002",
            "timestamp_ms": 1710000000100,
            "source": "ros2",
            "command_type": "arm",
            "payload": {
                "joint_positions_rad": [0.0, -0.5, 1.0, 0.0, 0.5, 0.0],
            },
            "metadata": {},
        },
        "gripper": {
            "protocol_version": PROTOCOL_VERSION,
            "command_id": "cmd-003",
            "timestamp_ms": 1710000000200,
            "source": "terminal",
            "command_type": "gripper",
            "payload": {
                "mode": "position",
                "position_rad": 0.25,
                "speed_ratio": 0.8,
            },
            "metadata": {},
        },
        "system": {
            "protocol_version": PROTOCOL_VERSION,
            "command_id": "cmd-004",
            "timestamp_ms": 1710000000300,
            "source": "terminal",
            "command_type": "system",
            "payload": {
                "operation": "pause",
                "reason": "manual safety pause",
            },
            "metadata": {},
        },
    }
