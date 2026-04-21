"""Log replay engine for deterministic command re-execution."""

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional
import json
import os
import time


@dataclass
class ReplayResult:
    replayed_commands: int
    dropped_commands: int
    avg_state_delta: float
    max_state_delta: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "replayed_commands": self.replayed_commands,
            "dropped_commands": self.dropped_commands,
            "avg_state_delta": self.avg_state_delta,
            "max_state_delta": self.max_state_delta,
        }



def _read_jsonl(path: str) -> List[Dict[str, Any]]:
    if not os.path.exists(path):
        return []
    rows: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


class ReplayEngine:
    def __init__(self, log_dir: str) -> None:
        self.log_dir = log_dir
        self.commands = _read_jsonl(os.path.join(log_dir, "validation.jsonl"))
        self.states = _read_jsonl(os.path.join(log_dir, "state.jsonl"))
        self.events = _read_jsonl(os.path.join(log_dir, "events.jsonl"))
        self.timeline = _read_jsonl(os.path.join(log_dir, "timeline_index.jsonl"))

    def replay(
        self,
        callback: Optional[Callable[[Dict[str, Any]], Optional[Dict[str, Any]]]] = None,
        headless: bool = True,
        speed: float = 1.0,
    ) -> ReplayResult:
        replayed = 0
        dropped = 0
        deltas: List[float] = []

        for row in self.commands:
            command_id = row.get("command_id", "")
            ok = bool(row.get("ok", False))
            normalized = row.get("normalized")

            if not ok or not normalized:
                dropped += 1
                continue

            replayed += 1
            if callback is not None:
                result = callback(row)
                if isinstance(result, dict):
                    delta = float(result.get("state_delta", 0.0))
                    deltas.append(abs(delta))

            # Use recorded pacing unless running as fast as possible.
            if speed > 0 and not headless:
                time.sleep(max(0.0, 0.001 / speed))

        avg_delta = sum(deltas) / len(deltas) if deltas else 0.0
        max_delta = max(deltas) if deltas else 0.0
        return ReplayResult(
            replayed_commands=replayed,
            dropped_commands=dropped,
            avg_state_delta=avg_delta,
            max_state_delta=max_delta,
        )
