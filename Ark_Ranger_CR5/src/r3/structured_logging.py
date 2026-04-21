"""Structured JSONL logging for end-to-end command traceability."""

from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from typing import Any, Dict, Optional
import json
import os
import threading
import time


class StructuredLogger:
    def __init__(self, root_dir: str) -> None:
        utc_now = datetime.now(timezone.utc)
        self.session_id = utc_now.strftime("%Y%m%dT%H%M%SZ")
        self.session_dir = os.path.join(root_dir, self.session_id)
        os.makedirs(self.session_dir, exist_ok=True)

        self._paths = {
            "commands": os.path.join(self.session_dir, "commands.jsonl"),
            "validation": os.path.join(self.session_dir, "validation.jsonl"),
            "safety": os.path.join(self.session_dir, "safety.jsonl"),
            "ik": os.path.join(self.session_dir, "ik.jsonl"),
            "state": os.path.join(self.session_dir, "state.jsonl"),
            "events": os.path.join(self.session_dir, "events.jsonl"),
            "startup": os.path.join(self.session_dir, "startup.jsonl"),
        }
        self._lock = threading.Lock()

    @property
    def session_path(self) -> str:
        return self.session_dir

    def _serialize(self, obj: Any) -> Any:
        if is_dataclass(obj):
            return asdict(obj)
        if hasattr(obj, "to_dict") and callable(getattr(obj, "to_dict")):
            return obj.to_dict()
        return obj

    def _base_record(self, command_id: str, stage: str) -> Dict[str, Any]:
        now = datetime.now(timezone.utc)
        return {
            "session_id": self.session_id,
            "stage": stage,
            "command_id": command_id,
            "timestamp_utc": now.isoformat(),
            "timestamp_ms": int(now.timestamp() * 1000),
            "monotonic_s": time.monotonic(),
        }

    def log(self, stream: str, command_id: str, stage: str, payload: Dict[str, Any]) -> None:
        if stream not in self._paths:
            return
        record = self._base_record(command_id=command_id, stage=stage)
        record.update({k: self._serialize(v) for k, v in payload.items()})
        line = json.dumps(record, ensure_ascii=False)
        with self._lock:
            with open(self._paths[stream], "a", encoding="utf-8") as fh:
                fh.write(line + "\n")

    def log_command(self, command_id: str, source: str, raw: Any) -> None:
        self.log("commands", command_id, "command.received", {"source": source, "raw": raw})

    def log_validation(self, command_id: str, ok: bool, issues: Any, normalized: Any) -> None:
        self.log(
            "validation",
            command_id,
            "command.validated",
            {
                "ok": ok,
                "issues": issues,
                "normalized": normalized,
            },
        )

    def log_safety(self, command_id: str, decision: str, reason_codes: Any, details: Dict[str, Any]) -> None:
        self.log(
            "safety",
            command_id,
            "safety.filtered",
            {
                "decision": decision,
                "reason_codes": reason_codes,
                "details": details,
            },
        )

    def log_ik(self, command_id: str, info: Dict[str, Any]) -> None:
        self.log("ik", command_id, "ik.result", info)

    def log_state(self, command_id: str, state: Dict[str, Any]) -> None:
        self.log("state", command_id, "state.snapshot", state)

    def log_event(self, command_id: str, event: Any) -> None:
        self.log("events", command_id, "event", {"event": event})

    def log_startup(self, report: Dict[str, Any]) -> None:
        self.log("startup", command_id="startup", stage="startup.report", payload={"report": report})
