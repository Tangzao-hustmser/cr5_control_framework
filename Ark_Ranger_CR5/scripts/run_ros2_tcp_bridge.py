# -*- coding: utf-8 -*-
import argparse
import json
import socket
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from r3_msgs.action import Command as CommandAction
from r3_msgs.msg import CommandResult, RuntimeStatus, SafetyDecision
from r3_msgs.srv import AssetCheck, HealthCheck, SystemControl


class TcpJsonClient:
    def __init__(self, host: str, port: int, timeout_s: float = 3.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._sock = None

    def _connect(self) -> None:
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def send_json_line(self, payload: dict) -> None:
        data = json.dumps(payload, ensure_ascii=False)
        if not data.endswith("\n"):
            data += "\n"
        encoded = data.encode("utf-8")
        with self._lock:
            if self._sock is None:
                self._connect()
            try:
                self._sock.sendall(encoded)
            except Exception:
                self._close()
                raise

    def close(self) -> None:
        with self._lock:
            self._close()


def _now_ms() -> int:
    return int(time.time() * 1000)


def _ensure_command_id(command_id: str) -> str:
    if command_id:
        return command_id
    return f"cmd-{uuid.uuid4().hex[:12]}"


def _parse_json_field(raw: str, field_name: str):
    if not raw:
        return {}, None
    try:
        return json.loads(raw), None
    except Exception as exc:
        return None, f"{field_name} json decode failed: {exc}"


def _build_command_payload(cmd):
    command_id = _ensure_command_id(cmd.command_id)
    timestamp_ms = int(cmd.timestamp_ms) if int(cmd.timestamp_ms) > 0 else _now_ms()
    payload, err = _parse_json_field(cmd.payload_json, "payload_json")
    if err:
        return {}, err
    metadata, err = _parse_json_field(cmd.metadata_json, "metadata_json")
    if err:
        return {}, err
    if not cmd.command_type:
        return {}, "command_type is required"

    return {
        "protocol_version": cmd.protocol_version or "r3.v1",
        "command_id": command_id,
        "timestamp_ms": timestamp_ms,
        "source": cmd.source or "ros2_bridge",
        "command_type": cmd.command_type,
        "payload": payload or {},
        "metadata": metadata or {},
    }, None


class R3TcpBridge(Node):
    def __init__(self, host: str, port: int, action_name: str) -> None:
        super().__init__("r3_tcp_bridge")
        self._client = TcpJsonClient(host, port)
        self._action_server = ActionServer(
            self,
            CommandAction,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
        )
        self._services = []
        self._services.append(self.create_service(SystemControl, "/r3/pause", self._make_system_handler("pause")))
        self._services.append(self.create_service(SystemControl, "/r3/resume", self._make_system_handler("resume")))
        self._services.append(self.create_service(SystemControl, "/r3/reset", self._make_system_handler("reset")))
        self._services.append(self.create_service(SystemControl, "/r3/stop", self._make_system_handler("stop")))
        self._services.append(self.create_service(HealthCheck, "/r3/health", self._handle_health))
        self._services.append(self.create_service(AssetCheck, "/r3/asset_check", self._handle_asset_check))

    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        cmd = goal_handle.request.command
        payload, err = _build_command_payload(cmd)
        result = CommandResult()
        result.command_id = _ensure_command_id(cmd.command_id)
        result.timestamp_ms = _now_ms()

        if err:
            result.ok = False
            result.status = "invalid_goal"
            result.message = err
            result.details_json = json.dumps({"error": err}, ensure_ascii=False)
            goal_handle.abort()
            return result

        feedback = CommandAction.Feedback()
        feedback.status = RuntimeStatus(
            command_id=result.command_id,
            timestamp_ms=result.timestamp_ms,
            step_idx=0,
            current_arm_rad=[],
            target_arm_rad=[],
            ee_pose_wxyz=[],
            gripper_position_rad=0.0,
            stage="bridge_sent",
            info_json=json.dumps({"note": "sent to tcp"}, ensure_ascii=False),
        )
        feedback.safety = SafetyDecision(decision="unknown", reason_codes=[], details_json="{}")
        feedback.stage = "bridge_sent"
        goal_handle.publish_feedback(feedback)

        try:
            self._client.send_json_line(payload)
            result.ok = True
            result.status = "sent"
            result.message = "sent to tcp"
            goal_handle.succeed()
        except Exception as exc:
            result.ok = False
            result.status = "bridge_send_failed"
            result.message = str(exc)
            result.details_json = json.dumps({"error": str(exc)}, ensure_ascii=False)
            goal_handle.abort()

        return result

    def _make_system_handler(self, operation: str):
        def _handler(request: SystemControl.Request, response: SystemControl.Response):
            command_id = _ensure_command_id("")
            timestamp_ms = _now_ms()
            payload = {
                "protocol_version": "r3.v1",
                "command_id": command_id,
                "timestamp_ms": timestamp_ms,
                "source": "ros2_bridge",
                "command_type": "system",
                "payload": {
                    "operation": operation,
                    "reason": request.reason or "",
                },
                "metadata": {},
            }
            try:
                self._client.send_json_line(payload)
                response.ok = True
                response.message = "sent to tcp"
            except Exception as exc:
                response.ok = False
                response.message = str(exc)
            response.timestamp_ms = timestamp_ms
            response.command_id = command_id
            return response

        return _handler

    def _handle_health(self, request: HealthCheck.Request, response: HealthCheck.Response):
        timestamp_ms = _now_ms()
        command_id = _ensure_command_id(request.request_id)
        payload = {
            "mode": "tcp_bridge_degraded",
            "reachable": True,
            "note": "Health check is reported by ROS2->TCP bridge, not Isaac ROS2 runtime.",
        }
        response.success = True
        response.status = "DEGRADED_BRIDGE"
        response.report_json = json.dumps(payload, ensure_ascii=False)
        response.timestamp_ms = timestamp_ms
        response.command_id = command_id
        return response

    def _handle_asset_check(self, request: AssetCheck.Request, response: AssetCheck.Response):
        timestamp_ms = _now_ms()
        command_id = _ensure_command_id(request.request_id)
        payload = {
            "mode": "tcp_bridge_degraded",
            "reachable": True,
            "note": "Asset check unavailable in TCP bridge mode; query Windows-side reports directly.",
        }
        response.success = False
        response.status = "UNAVAILABLE_BRIDGE_MODE"
        response.report_json = json.dumps(payload, ensure_ascii=False)
        response.timestamp_ms = timestamp_ms
        response.command_id = command_id
        return response

    def shutdown(self) -> None:
        try:
            self._action_server.destroy()
        except Exception:
            pass
        for svc in self._services:
            try:
                self.destroy_service(svc)
            except Exception:
                pass
        self._services = []
        self._client.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS2 to TCP JSON bridge for Ark Ranger CR5")
    parser.add_argument("--host", default="127.0.0.1", help="Windows host IP or hostname")
    parser.add_argument("--port", type=int, default=58000, help="TCP JSON port exposed by Isaac Sim")
    parser.add_argument("--action-name", default="/r3/command", help="Action name to serve")
    args = parser.parse_args()

    rclpy.init()
    node = R3TcpBridge(args.host, args.port, args.action_name)
    node.get_logger().info(f"TCP bridge ready: {args.host}:{args.port} -> {args.action_name}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
