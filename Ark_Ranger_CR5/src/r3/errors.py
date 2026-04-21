"""Stable fault/error code catalog for R3 command pipeline."""

from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class FaultSpec:
    code: str
    name: str
    message: str
    severity: str


FAULT_SPECS = {
    "R3-E1001": FaultSpec("R3-E1001", "INVALID_JSON", "Input is not valid JSON.", "error"),
    "R3-E1002": FaultSpec("R3-E1002", "INVALID_MESSAGE_TYPE", "Message must be a dictionary object.", "error"),
    "R3-E1003": FaultSpec("R3-E1003", "UNSUPPORTED_PROTOCOL_VERSION", "Protocol version is not supported.", "error"),
    "R3-E1101": FaultSpec("R3-E1101", "MISSING_REQUIRED_FIELD", "Required field is missing.", "error"),
    "R3-E1102": FaultSpec("R3-E1102", "UNKNOWN_FIELD", "Message contains unsupported field.", "error"),
    "R3-E1103": FaultSpec("R3-E1103", "FIELD_TYPE_ERROR", "Field type is invalid.", "error"),
    "R3-E1201": FaultSpec("R3-E1201", "DIMENSION_MISMATCH", "Vector dimension does not match runtime expectation.", "error"),
    "R3-E1202": FaultSpec("R3-E1202", "VALUE_OUT_OF_RANGE", "Value is outside allowed range.", "error"),
    "R3-E1203": FaultSpec("R3-E1203", "INVALID_UNIT_FIELD", "Payload unit field does not match protocol.", "error"),
    "R3-E1204": FaultSpec("R3-E1204", "INVALID_COORDINATE_FRAME", "Coordinate frame is not supported.", "error"),
    "R3-E1301": FaultSpec("R3-E1301", "UNSUPPORTED_COMMAND_TYPE", "Command type is not supported.", "error"),
    "R3-E1401": FaultSpec("R3-E1401", "SAFETY_COMMAND_BLOCKED", "Command blocked by safety filter.", "error"),
    "R3-E1402": FaultSpec("R3-E1402", "SAFETY_COMMAND_CLAMPED", "Command was clamped by safety filter.", "warning"),
    "R3-E1501": FaultSpec("R3-E1501", "IK_SOLVE_FAILED", "Inverse kinematics solver failed.", "warning"),
    "R3-E1502": FaultSpec("R3-E1502", "IK_RETRY_EXCEEDED", "IK retry budget exhausted, fallback applied.", "error"),
    "R3-E1601": FaultSpec("R3-E1601", "GRIPPER_STALL_DETECTED", "Gripper appears stalled.", "warning"),
    "R3-E1602": FaultSpec("R3-E1602", "GRIPPER_TARGET_OUT_OF_RANGE", "Gripper target is outside limits.", "error"),
    "R3-E1701": FaultSpec("R3-E1701", "ADAPTER_NOT_AVAILABLE", "Requested input adapter is not available.", "error"),
    "R3-E1702": FaultSpec("R3-E1702", "ROS2_DEPENDENCY_MISSING", "ROS2 dependencies are not available.", "error"),
    "R3-E1801": FaultSpec("R3-E1801", "ASSET_CONSISTENCY_FAILED", "Asset consistency check failed.", "error"),
    "R3-E1901": FaultSpec("R3-E1901", "STARTUP_HEALTH_BLOCKED", "Startup health check blocked launch.", "error"),
    "R3-E1902": FaultSpec("R3-E1902", "STARTUP_HEALTH_DEGRADED", "Startup health check entered degraded mode.", "warning"),
    "R3-E1999": FaultSpec("R3-E1999", "UNKNOWN_INTERNAL_ERROR", "Unhandled internal error.", "error"),
}


def fault_dict() -> Dict[str, Dict[str, str]]:
    """Return fault dictionary for documentation and responses."""
    return {
        code: {
            "name": spec.name,
            "message": spec.message,
            "severity": spec.severity,
        }
        for code, spec in FAULT_SPECS.items()
    }


def get_fault_spec(code: str) -> FaultSpec:
    return FAULT_SPECS.get(code, FAULT_SPECS["R3-E1999"])
