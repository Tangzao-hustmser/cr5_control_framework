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
        self._next_index = 0
        self._video: List[Tuple[int, str, Dict[str, Any], int]] = []
        self._contact: List[Tuple[int, str, Dict[str, Any], int]] = []
        self._state: List[Tuple[int, str, Dict[str, Any], int]] = []

    def _append(self, record: TimelineRecord) -> int:
        with self._lock:
            timeline_index = self._next_index
            self._next_index += 1
            payload = record.to_dict()
            payload["timeline_index"] = timeline_index
            with open(self.path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(payload, ensure_ascii=False) + "\n")
        return timeline_index

    def register_video_frame(self, frame_id: str, timestamp_ms: int, frame_info: Optional[Dict[str, Any]] = None) -> int:
        payload = frame_info or {}
        timeline_index = self._append(TimelineRecord(kind="video", timestamp_ms=timestamp_ms, ref_id=frame_id, payload=payload))
        self._video.append((timestamp_ms, frame_id, payload, timeline_index))
        return timeline_index

    def register_contact(self, contact_id: str, timestamp_ms: int, contact_info: Optional[Dict[str, Any]] = None) -> int:
        payload = contact_info or {}
        timeline_index = self._append(TimelineRecord(kind="contact", timestamp_ms=timestamp_ms, ref_id=contact_id, payload=payload))
        self._contact.append((timestamp_ms, contact_id, payload, timeline_index))
        return timeline_index

    def register_state_snapshot(self, state_id: str, timestamp_ms: int, state_info: Optional[Dict[str, Any]] = None) -> int:
        payload = state_info or {}
        timeline_index = self._append(TimelineRecord(kind="state", timestamp_ms=timestamp_ms, ref_id=state_id, payload=payload))
        self._state.append((timestamp_ms, state_id, payload, timeline_index))
        return timeline_index

    def _nearest(self, pool: List[Tuple[int, str, Dict[str, Any], int]], timestamp_ms: int) -> Optional[Dict[str, Any]]:
        if not pool:
            return None
        best = min(pool, key=lambda item: abs(item[0] - timestamp_ms))
        return {
            "timestamp_ms": best[0],
            "ref_id": best[1],
            "payload": best[2],
            "timeline_index": best[3],
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
        mapping["timeline_index"] = self._append(
            TimelineRecord(kind="event_index", timestamp_ms=timestamp_ms, ref_id=event_id, payload=mapping)
        )
        return mapping
