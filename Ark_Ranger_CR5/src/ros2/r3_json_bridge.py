"""Compatibility bridge: JSON/terminal input -> ROS2 Action/Service."""

from __future__ import annotations

from typing import Any, Dict, Optional
import json
import time

import rclpy
from rclpy.action import ActionClient

from r3_msgs.action import Command
from r3_msgs.msg import Command as CommandMsg
from r3_msgs.srv import SystemControl

from src.r3.adapters import LocalInputAdapter
from src.r3.errors import get_fault_spec


class JsonToRos2Bridge:
    def __init__(self, prompt: str = "r3-json> ", show_help: bool = True, action_name: str = "/r3/command") -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("r3_json_bridge")
        self._client = ActionClient(self._node, Command, action_name)
        self._system_clients = {
            "pause": self._node.create_client(SystemControl, "/r3/pause"),
            "resume": self._node.create_client(SystemControl, "/r3/resume"),
            "reset": self._node.create_client(SystemControl, "/r3/reset"),
            "stop": self._node.create_client(SystemControl, "/r3/stop"),
        }
        self._adapter = LocalInputAdapter(prompt=prompt, show_help=show_help)
        self._adapter.start()

    def _make_goal(self, raw: Dict[str, Any]) -> Command.Goal:
        goal = Command.Goal()
        msg = CommandMsg()
        msg.protocol_version = str(raw.get("protocol_version", "r3.v1"))
        msg.command_id = str(raw.get("command_id", ""))
        msg.timestamp_ms = int(raw.get("timestamp_ms", int(time.time() * 1000)))
        msg.source = str(raw.get("source", "json_bridge"))
        msg.command_type = str(raw.get("command_type", ""))
        msg.payload_json = json.dumps(raw.get("payload", {}), ensure_ascii=False)
        msg.metadata_json = json.dumps(raw.get("metadata", {}), ensure_ascii=False)
        goal.command = msg
        return goal

    def _send_system(self, operation: str, reason: str = "") -> None:
        client = self._system_clients.get(operation)
        if client is None:
            print(f"[Bridge] Unsupported system operation: {operation}")
            return
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[Bridge] Service /r3/{operation} not available")
            return
        req = SystemControl.Request()
        req.operation = operation
        req.reason = reason or ""
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        if future.result() is not None:
            resp = future.result()
            print(f"[Bridge] system.{operation} -> ok={resp.ok} msg={resp.message}")
        else:
            print(f"[Bridge] system.{operation} failed")

    def spin(self) -> None:
        print("[Bridge] JSON->ROS2 bridge running. Use r3-json prompt to send commands.")
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            for msg in self._adapter.poll():
                raw = msg.raw
                if isinstance(raw, dict) and "__adapter_error__" in raw:
                    err = raw["__adapter_error__"]
                    print(f"[Bridge] input rejected: {err.get('message', '')}")
                    continue
                if not isinstance(raw, dict):
                    spec = get_fault_spec("R3-E1002")
                    print(f"[Bridge] input rejected: {spec.message}")
                    continue
                if raw.get("command_type") == "system":
                    payload = raw.get("payload", {}) if isinstance(raw.get("payload"), dict) else {}
                    operation = str(payload.get("operation", ""))
                    reason = str(payload.get("reason", ""))
                    self._send_system(operation, reason)
                    continue
                if not self._client.wait_for_server(timeout_sec=2.0):
                    print("[Bridge] Action server /r3/command not available")
                    continue
                goal = self._make_goal(raw)
                send_future = self._client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=5.0)
                goal_handle = send_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    print("[Bridge] Goal rejected")
                    continue
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=5.0)
                result = result_future.result()
                if result is not None:
                    res = result.result
                    print(f"[Bridge] result: ok={res.ok} status={res.status} fault={res.fault_code}")

    def shutdown(self) -> None:
        self._adapter.stop()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

