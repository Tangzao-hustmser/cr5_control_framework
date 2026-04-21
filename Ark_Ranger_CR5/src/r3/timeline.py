"""Timeline index linking video/contact/state timestamps to events."""

from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Optional, Tuple
import json
import os
import threading


@dataclass
class TimelineRecord:
    kind: str
    timestamp_ms: int
    ref_id: str
    payload: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class TimelineIndexer:
    def __init__(self, output_dir: str) -> None:
        os.makedirs(output_dir, exist_ok=True)
        self.path = os.path.join(output_dir, "timeline_index.jsonl")
        self._lock = threading.Lock()
        self._video: List[Tuple[int, str, Dict[str, Any]]] = []
        self._contact: List[Tuple[int, str, Dict[str, Any]]] = []
        self._state: List[Tuple[int, str, Dict[str, Any]]] = []

    def _append(self, record: TimelineRecord) -> None:
        with self._lock:
            with open(self.path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(record.to_dict(), ensure_ascii=False) + "\n")

    def register_video_frame(self, frame_id: str, timestamp_ms: int, frame_info: Optional[Dict[str, Any]] = None) -> None:
        payload = frame_info or {}
        self._video.append((timestamp_ms, frame_id, payload))
        self._append(TimelineRecord(kind="video", timestamp_ms=timestamp_ms, ref_id=frame_id, payload=payload))

    def register_contact(self, contact_id: str, timestamp_ms: int, contact_info: Optional[Dict[str, Any]] = None) -> None:
        payload = contact_info or {}
        self._contact.append((timestamp_ms, contact_id, payload))
        self._append(TimelineRecord(kind="contact", timestamp_ms=timestamp_ms, ref_id=contact_id, payload=payload))

    def register_state_snapshot(self, state_id: str, timestamp_ms: int, state_info: Optional[Dict[str, Any]] = None) -> None:
        payload = state_info or {}
        self._state.append((timestamp_ms, state_id, payload))
        self._append(TimelineRecord(kind="state", timestamp_ms=timestamp_ms, ref_id=state_id, payload=payload))

    def _nearest(self, pool: List[Tuple[int, str, Dict[str, Any]]], timestamp_ms: int) -> Optional[Dict[str, Any]]:
        if not pool:
            return None
        best = min(pool, key=lambda item: abs(item[0] - timestamp_ms))
        return {
            "timestamp_ms": best[0],
            "ref_id": best[1],
            "payload": best[2],
        }

    def link_event(self, event_id: str, timestamp_ms: int, command_id: str, reason_code: str) -> Dict[str, Any]:
        mapping = {
            "event_id": event_id,
            "timestamp_ms": timestamp_ms,
            "command_id": command_id,
            "reason_code": reason_code,
            "nearest_video": self._nearest(self._video, timestamp_ms),
            "nearest_contact": self._nearest(self._contact, timestamp_ms),
            "nearest_state": self._nearest(self._state, timestamp_ms),
        }
        self._append(TimelineRecord(kind="event_index", timestamp_ms=timestamp_ms, ref_id=event_id, payload=mapping))
        return mapping
