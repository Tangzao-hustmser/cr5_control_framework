import os
import sys
import time

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

import ark_patch  # noqa: F401
from ark_patch import Node


class ArkMessage:
    def __init__(self, data):
        self.data = data


class SimpleTestPolicy(Node):
    def __init__(self):
        super().__init__("test_policy_node")
        self.pub = type("obj", (object,), {"publish": lambda _, msg: print(f"publish: {msg.data}")})()

    def send_command(self):
        msg = ArkMessage(
            data={
                "protocol_version": "r3.v1",
                "command_id": "test-cmd-001",
                "timestamp_ms": int(time.time() * 1000),
                "source": "test_policy",
                "command_type": "arm",
                "payload": {
                    "joint_positions_rad": [0.0, -0.5, 1.0, 0.0, 0.5, 0.0],
                },
                "metadata": {},
            }
        )
        self.pub.publish(msg)


if __name__ == "__main__":
    policy = SimpleTestPolicy()
    print("--- test policy running with unified protocol ---")
    while True:
        policy.send_command()
        time.sleep(2)
