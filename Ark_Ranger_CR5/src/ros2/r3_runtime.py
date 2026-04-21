"""ROS2 runtime wiring for Ark Ranger R3."""

from __future__ import annotations

from typing import Any, Dict, Optional
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .r3_action_server import R3ActionServer
from .r3_publishers import R3Ros2Publishers
from .r3_services import R3Services


class R3Ros2Runtime:
    def __init__(
        self,
        isaac_node: Any,
        project_root: str,
        robot_cfg_path: str,
        runtime_cfg_path: str,
        robot_cfg: Dict[str, Any],
        runtime_cfg: Dict[str, Any],
    ) -> None:
        self._isaac_node = isaac_node
        self._runtime_cfg = runtime_cfg
        ros2_cfg = runtime_cfg.get("ros2", {})
        node_name = str(ros2_cfg.get("node_name", "ark_ranger_r3"))
        action_name = str(ros2_cfg.get("action_name", "/r3/command"))
        action_cfg = ros2_cfg.get("action", {})

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node(node_name)
        self.publishers = R3Ros2Publishers(self.node, runtime_cfg)
        self.action_server = R3ActionServer(
            self.node,
            isaac_node,
            self.publishers,
            action_name=action_name,
            action_cfg=action_cfg,
        )
        self.services = R3Services(
            self.node,
            isaac_node,
            project_root,
            robot_cfg_path,
            runtime_cfg_path,
            robot_cfg,
            runtime_cfg,
        )
        isaac_node.attach_ros2_publishers(self.publishers)

        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

    def shutdown(self) -> None:
        try:
            self.action_server.shutdown()
        except Exception:
            pass
        try:
            self.services.shutdown()
        except Exception:
            pass
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            if self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass

