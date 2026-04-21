"""Utilities to map between r3_msgs and r3.v1 command dictionaries."""

from __future__ import annotations

import json
import time
from typing import Any, Dict

from src.r3.protocol import PROTOCOL_VERSION, generate_command_id


def _now_ms() -> int:
    return int(time.time() * 1000)


def _safe_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _safe_json_load(text: str) -> Dict[str, Any]:
    if not text:
        return {}
    try:
        data = json.loads(text)
    except Exception:
        return {}
    return _safe_dict(data)


def command_dict_to_msg(raw: Dict[str, Any]) -> Any:
    from r3_msgs.msg import Command

    payload = _safe_dict(raw.get("payload"))
    metadata = _safe_dict(raw.get("metadata"))

    msg = Command()
    msg.protocol_version = str(raw.get("protocol_version") or PROTOCOL_VERSION)
    msg.command_id = str(raw.get("command_id") or generate_command_id())
    msg.timestamp_ms = int(raw.get("timestamp_ms") or _now_ms())
    msg.source = str(raw.get("source") or "ros2")
    msg.command_type = str(raw.get("command_type") or "")
    msg.payload_json = json.dumps(payload, ensure_ascii=False)
    msg.metadata_json = json.dumps(metadata, ensure_ascii=False)
    return msg


def command_msg_to_dict(msg: Any, default_source: str = "ros2") -> Dict[str, Any]:
    return {
        "protocol_version": str(msg.protocol_version or PROTOCOL_VERSION),
        "command_id": str(msg.command_id or generate_command_id()),
        "timestamp_ms": int(msg.timestamp_ms or _now_ms()),
        "source": str(msg.source or default_source),
        "command_type": str(msg.command_type or ""),
        "payload": _safe_json_load(msg.payload_json),
        "metadata": _safe_json_load(msg.metadata_json),
    }
