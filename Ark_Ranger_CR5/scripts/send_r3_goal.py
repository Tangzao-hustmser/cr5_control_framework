#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Send /r3/command action goals with short CLI options.

Examples:
  python scripts/send_r3_goal.py pose --x 0.45 --y 0.15 --z 0.80 --feedback
  python scripts/send_r3_goal.py arm --joints 0 -0.5 1.0 0 0.5 0 --feedback
  python scripts/send_r3_goal.py gripper --mode open
  python scripts/send_r3_goal.py system --operation stop --reason manual
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid
from typing import Any, Dict, List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from r3_msgs.action import Command as CommandAction
from r3_msgs.msg import Command as CommandMsg


def _now_ms() -> int:
    return int(time.time() * 1000)


def _gen_command_id() -> str:
    return f"cmd-{uuid.uuid4().hex[:12]}"


def _ensure_json_obj(raw: str, field_name: str) -> Dict[str, Any]:
    try:
        data = json.loads(raw) if raw else {}
    except json.JSONDecodeError as exc:
        raise ValueError(f"{field_name} json decode failed: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"{field_name} must be a JSON object")
    return data


def _build_payload_from_args(args: argparse.Namespace) -> Dict[str, Any]:
    if args.kind == "pose":
        return {
            "frame": args.frame,
            "position_m": [args.x, args.y, args.z],
            "orientation_wxyz": [args.qw, args.qx, args.qy, args.qz],
        }
    if args.kind == "arm":
        return {"joint_positions_rad": [float(v) for v in args.joints]}
    if args.kind == "gripper":
        payload: Dict[str, Any] = {"mode": args.mode}
        if args.mode == "position":
            if args.position_rad is None:
                raise ValueError("--position-rad is required when --mode position")
            payload["position_rad"] = float(args.position_rad)
        elif args.position_rad is not None:
            payload["position_rad"] = float(args.position_rad)
        return payload
    if args.kind == "system":
        return {"operation": args.operation, "reason": args.reason or ""}
    if args.kind == "raw":
        return _ensure_json_obj(args.payload_json, "payload_json")
    raise ValueError(f"unsupported command kind: {args.kind}")


def _resolve_command_type(args: argparse.Namespace) -> str:
    if args.kind == "raw":
        return str(args.command_type)
    return str(args.kind)


class R3GoalSender(Node):
    def __init__(self, action_name: str) -> None:
        super().__init__("r3_goal_sender")
        self._client = ActionClient(self, CommandAction, action_name)

    def wait_server(self, timeout_s: float) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_s)

    def send_goal(self, goal: CommandAction.Goal, show_feedback: bool, timeout_s: float) -> CommandAction.Result:
        def _feedback_cb(msg: CommandAction.FeedbackMessage) -> None:
            if not show_feedback:
                return
            fb = msg.feedback
            stage = getattr(fb, "stage", "")
            status = getattr(fb, "status", None)
            status_stage = getattr(status, "stage", "") if status is not None else ""
            self.get_logger().info(f"feedback stage={stage} status.stage={status_stage}")

        future = self._client.send_goal_async(goal, feedback_callback=_feedback_cb if show_feedback else None)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        goal_handle = future.result()
        if goal_handle is None:
            raise RuntimeError("failed to send goal: timeout")
        if not goal_handle.accepted:
            raise RuntimeError("goal rejected")

        self.get_logger().info("goal accepted, waiting result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=max(timeout_s, 0.1))
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("wait result timeout")
        return wrapped.result


def _build_goal(args: argparse.Namespace) -> CommandAction.Goal:
    payload = _build_payload_from_args(args)
    metadata = _ensure_json_obj(args.metadata_json, "metadata_json")

    cmd = CommandMsg()
    cmd.protocol_version = args.protocol_version
    cmd.command_id = args.command_id or _gen_command_id()
    cmd.timestamp_ms = int(args.timestamp_ms) if args.timestamp_ms > 0 else _now_ms()
    cmd.source = args.source
    cmd.command_type = _resolve_command_type(args)
    cmd.payload_json = json.dumps(payload, ensure_ascii=False)
    cmd.metadata_json = json.dumps(metadata, ensure_ascii=False)

    goal = CommandAction.Goal()
    goal.command = cmd
    return goal


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send /r3/command goals with short options")
    parser.add_argument("--action-name", default="/r3/command", help="Action name")
    parser.add_argument("--protocol-version", default="r3.v1", help="Protocol version")
    parser.add_argument("--command-id", default="", help="Optional command id")
    parser.add_argument("--timestamp-ms", type=int, default=0, help="Optional timestamp in ms, default now")
    parser.add_argument("--source", default="ros2_cli_short", help="Command source field")
    parser.add_argument("--metadata-json", default="{}", help="Metadata JSON object string")
    parser.add_argument("--wait-server-timeout", type=float, default=5.0, help="Wait action server timeout (sec)")
    parser.add_argument("--wait-result-timeout", type=float, default=20.0, help="Wait result timeout (sec)")
    parser.add_argument("--feedback", action="store_true", help="Print feedback stage")
    parser.add_argument("--dry-run", action="store_true", help="Only print goal JSON and exit")

    sub = parser.add_subparsers(dest="kind", required=True)

    pose = sub.add_parser("pose", help="Send pose command")
    pose.add_argument("--frame", default="world", help="Pose frame")
    pose.add_argument("--x", type=float, required=True, help="Position x (m)")
    pose.add_argument("--y", type=float, required=True, help="Position y (m)")
    pose.add_argument("--z", type=float, required=True, help="Position z (m)")
    pose.add_argument("--qw", type=float, default=1.0, help="Orientation w")
    pose.add_argument("--qx", type=float, default=0.0, help="Orientation x")
    pose.add_argument("--qy", type=float, default=0.0, help="Orientation y")
    pose.add_argument("--qz", type=float, default=0.0, help="Orientation z")

    arm = sub.add_parser("arm", help="Send arm joints command")
    arm.add_argument("--joints", nargs="+", type=float, required=True, help="Joint radians list")

    gripper = sub.add_parser("gripper", help="Send gripper command")
    gripper.add_argument("--mode", choices=["open", "close", "position"], required=True, help="Gripper mode")
    gripper.add_argument("--position-rad", type=float, default=None, help="Gripper position rad")

    system = sub.add_parser("system", help="Send system command through action")
    system.add_argument("--operation", choices=["pause", "resume", "reset", "stop"], required=True, help="Operation")
    system.add_argument("--reason", default="", help="Operation reason")

    raw = sub.add_parser("raw", help="Send custom command_type + payload_json")
    raw.add_argument("--command-type", required=True, help="Command type")
    raw.add_argument("--payload-json", default="{}", help="Payload JSON object string")

    return parser


def main() -> int:
    parser = _make_parser()
    args = parser.parse_args()

    try:
        goal = _build_goal(args)
    except ValueError as exc:
        print(f"[r3-send] invalid arguments: {exc}", file=sys.stderr)
        return 2

    goal_preview = {
        "command": {
            "protocol_version": goal.command.protocol_version,
            "command_id": goal.command.command_id,
            "timestamp_ms": int(goal.command.timestamp_ms),
            "source": goal.command.source,
            "command_type": goal.command.command_type,
            "payload_json": goal.command.payload_json,
            "metadata_json": goal.command.metadata_json,
        }
    }
    print("[r3-send] goal:")
    print(json.dumps(goal_preview, ensure_ascii=False, indent=2))
    if args.dry_run:
        return 0

    rclpy.init(args=None)
    node = R3GoalSender(args.action_name)
    try:
        if not node.wait_server(args.wait_server_timeout):
            print(f"[r3-send] action server not available: {args.action_name}", file=sys.stderr)
            return 3
        result = node.send_goal(goal, show_feedback=bool(args.feedback), timeout_s=float(args.wait_result_timeout))
        result_dict: Dict[str, Any] = {
            "command_id": result.command_id,
            "timestamp_ms": int(result.timestamp_ms),
            "ok": bool(result.ok),
            "status": result.status,
            "fault_code": result.fault_code,
            "message": result.message,
            "details_json": result.details_json,
        }
        print("[r3-send] result:")
        print(json.dumps(result_dict, ensure_ascii=False, indent=2))
        return 0 if bool(result.ok) else 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
