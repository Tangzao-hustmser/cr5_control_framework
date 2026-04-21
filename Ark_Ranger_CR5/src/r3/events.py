"""Event type and event publishing helpers for R3 pipeline."""

from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional


EVENT_TYPES = {
    "COMMAND_RECEIVED": "command.received",
    "COMMAND_REJECTED": "command.rejected",
    "COMMAND_NORMALIZED": "command.normalized",
    "SAFETY_PASS": "safety.pass",
    "SAFETY_CLAMP": "safety.clamp",
    "SAFETY_BLOCK": "safety.block",
    "IK_SUCCESS": "ik.success",
    "IK_FAILURE": "ik.failure",
    "IK_FALLBACK": "ik.fallback",
    "GRIPPER_STALL": "gripper.stall",
    "CONTROL_APPLIED": "control.applied",
    "SYSTEM_STATE_CHANGE": "system.state_change",
    "STARTUP_REPORT": "startup.report",
}


@dataclass
class SystemEvent:
    event_type: str
    command_id: str
    fault_code: str
    reason: str
    details: Dict[str, Any]
    timestamp_ms: int

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class EventPublisher:
    """Simple in-process event publisher used by logger/indexer hooks."""

    def __init__(self) -> None:
        self._subscribers: List[Callable[[SystemEvent], None]] = []

    def subscribe(self, callback: Callable[[SystemEvent], None]) -> None:
        self._subscribers.append(callback)

    def publish(
        self,
        event_type: str,
        command_id: str,
        fault_code: str,
        reason: str,
        details: Optional[Dict[str, Any]] = None,
    ) -> SystemEvent:
        now = datetime.now(timezone.utc)
        event = SystemEvent(
            event_type=event_type,
            command_id=command_id,
            fault_code=fault_code,
            reason=reason,
            details=details or {},
            timestamp_ms=int(now.timestamp() * 1000),
        )
        for callback in list(self._subscribers):
            callback(event)
        return event
