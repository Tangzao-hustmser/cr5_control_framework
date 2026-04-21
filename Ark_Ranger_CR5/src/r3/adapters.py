"""Input adapter abstraction for command ingestion."""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import json
import math
import queue
import threading
import time

from .errors import get_fault_spec


@dataclass
class AdapterMessage:
    source: str
    raw: Any
    received_ms: int


class InputAdapter:
    name = "base"

    def start(self) -> None:
        pass

    def poll(self) -> List[AdapterMessage]:
        return []

    def stop(self) -> None:
        pass

    def health(self) -> Dict[str, Any]:
        return {"name": self.name, "ready": True}


class LocalInputAdapter(InputAdapter):
    """Read unified JSON commands from terminal input."""

    name = "local_terminal"

    def __init__(self, prompt: str = "r3-json> ", show_help: bool = True) -> None:
        self.prompt = prompt
        self.show_help = show_help
        self._queue: "queue.Queue[AdapterMessage]" = queue.Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        if self.show_help:
            print("\n==================== Unified Command Console ====================")
            print("Input one command per line (JSON protocol r3.v1 or arm shortcut).")
            print("Example pose command:")
            print('{"command_type":"pose","payload":{"frame":"world","position_m":[0.4,0.1,0.5],"orientation_wxyz":[1,0,0,0]}}')
            print("Shortcut arm command in degrees (recommended):")
            print("arm 0 -30 45 0 0 0")
            print("Shortcut arm command in radians:")
            print("arm_rad 0 -0.52 0.79 0 0 0")
            print("================================================================")
        while self._running:
            try:
                line = input(self.prompt)
            except EOFError:
                break
            except Exception:
                time.sleep(0.05)
                continue
            if not line:
                continue
            self._queue.put(AdapterMessage(source=self.name, raw=line, received_ms=int(time.time() * 1000)))

    def _parse_arm_shortcut(self, line: str) -> Optional[Dict[str, Any]]:
        text = str(line).strip()
        lower = text.lower()

        if lower == "arm_rad":
            values_text = ""
            unit = "rad"
        elif lower.startswith("arm_rad "):
            values_text = text[len("arm_rad "):].strip()
            unit = "rad"
        elif lower == "arm":
            values_text = ""
            unit = "deg"
        elif lower.startswith("arm "):
            values_text = text[len("arm "):].strip()
            unit = "deg"
        else:
            return None

        if not values_text:
            raise ValueError("missing joint angle values")

        normalized = values_text.replace(",", " ")
        tokens = [tok for tok in normalized.split() if tok]
        try:
            values = [float(tok) for tok in tokens]
        except ValueError:
            raise ValueError("joint angle values must be numeric")

        if unit == "deg":
            values = [math.radians(v) for v in values]

        return {
            "command_type": "arm",
            "payload": {
                "joint_positions_rad": values,
            },
            "metadata": {
                "terminal_shortcut": f"arm_{unit}",
            },
        }

    def poll(self) -> List[AdapterMessage]:
        out: List[AdapterMessage] = []
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item.raw, str):
                raw_line = item.raw.strip()
                try:
                    shortcut = self._parse_arm_shortcut(raw_line)
                except ValueError as exc:
                    spec = get_fault_spec("R3-E1001")
                    out.append(
                        AdapterMessage(
                            source=item.source,
                            raw={
                                "__adapter_error__": {
                                    "fault_code": spec.code,
                                    "message": f"{spec.message} detail=arm shortcut parse failed: {str(exc)}",
                                    "raw_line": item.raw,
                                }
                            },
                            received_ms=item.received_ms,
                        )
                    )
                    continue

                if shortcut is not None:
                    out.append(AdapterMessage(source=item.source, raw=shortcut, received_ms=item.received_ms))
                    continue

                try:
                    raw = json.loads(item.raw)
                    out.append(AdapterMessage(source=item.source, raw=raw, received_ms=item.received_ms))
                except json.JSONDecodeError as exc:
                    spec = get_fault_spec("R3-E1001")
                    out.append(
                        AdapterMessage(
                            source=item.source,
                            raw={
                                "__adapter_error__": {
                                    "fault_code": spec.code,
                                    "message": f"{spec.message} detail={str(exc)}",
                                    "raw_line": item.raw,
                                }
                            },
                            received_ms=item.received_ms,
                        )
                    )
            else:
                out.append(item)
        return out

    def stop(self) -> None:
        self._running = False


class Ros2InputAdapter(InputAdapter):
    """Read unified JSON commands from ROS2 string topic."""

    name = "ros2"

    def __init__(self, topic: str = "/robot/action", node_name: str = "ark_ranger_r3_adapter") -> None:
        self.topic = topic
        self.node_name = node_name
        self._ready = False
        self._queue: "queue.Queue[AdapterMessage]" = queue.Queue()
        self._rclpy = None
        self._node = None

    def start(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except Exception as exc:
            raise RuntimeError(f"ROS2 unavailable: {exc}")

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node(self.node_name)

        def _on_msg(msg: Any) -> None:
            received_ms = int(time.time() * 1000)
            try:
                payload = json.loads(msg.data)
                self._queue.put(AdapterMessage(source=self.name, raw=payload, received_ms=received_ms))
            except Exception as exc:
                spec = get_fault_spec("R3-E1001")
                self._queue.put(
                    AdapterMessage(
                        source=self.name,
                        raw={
                            "__adapter_error__": {
                                "fault_code": spec.code,
                                "message": f"{spec.message} detail={str(exc)}",
                                "raw_line": getattr(msg, "data", ""),
                            }
                        },
                        received_ms=received_ms,
                    )
                )

        self._node.create_subscription(String, self.topic, _on_msg, 10)
        self._ready = True

    def poll(self) -> List[AdapterMessage]:
        out: List[AdapterMessage] = []
        if self._ready and self._rclpy is not None and self._node is not None:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)
        while True:
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return out

    def stop(self) -> None:
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        if self._rclpy is not None:
            try:
                if self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass
        self._ready = False

    def health(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "ready": self._ready,
            "topic": self.topic,
        }


def build_adapters(runtime_cfg: Dict[str, Any]) -> List[InputAdapter]:
    adapters: List[InputAdapter] = []
    selected = runtime_cfg.get("input_source", "local_terminal")
    adapters_cfg = runtime_cfg.get("adapters", {})

    if selected == "local_terminal":
        cfg = adapters_cfg.get("local_terminal", {})
        adapters.append(
            LocalInputAdapter(
                prompt=str(cfg.get("prompt", "r3-json> ")),
                show_help=bool(cfg.get("help_on_start", True)),
            )
        )
    elif selected == "ros2":
        cfg = adapters_cfg.get("ros2", {})
        adapters.append(
            Ros2InputAdapter(
                topic=str(cfg.get("topic", "/robot/action")),
                node_name=str(cfg.get("node_name", "ark_ranger_r3_adapter")),
            )
        )
    else:
        raise RuntimeError(f"Unknown input_source: {selected}")

    return adapters
