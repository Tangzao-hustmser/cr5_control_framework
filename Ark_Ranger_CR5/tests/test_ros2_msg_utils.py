import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import r3_msgs  # noqa: F401

    HAS_R3_MSGS = True
except Exception:
    HAS_R3_MSGS = False

from src.ros2.r3_msg_utils import command_dict_to_msg, command_msg_to_dict


@unittest.skipUnless(HAS_R3_MSGS, "r3_msgs not available")
class TestRos2MsgUtils(unittest.TestCase):
    def test_command_msg_roundtrip_defaults(self):
        raw = {
            "command_type": "pose",
            "payload": {
                "frame": "world",
                "position_m": [0.1, 0.2, 0.3],
                "orientation_wxyz": [1, 0, 0, 0],
            },
        }
        msg = command_dict_to_msg(raw)
        out = command_msg_to_dict(msg, default_source="ros2")

        self.assertEqual(out["command_type"], "pose")
        self.assertEqual(out["payload"]["position_m"], [0.1, 0.2, 0.3])
        self.assertEqual(out["payload"]["orientation_wxyz"], [1.0, 0.0, 0.0, 0.0])
        self.assertTrue(out["protocol_version"])
        self.assertTrue(out["command_id"])

    def test_command_msg_arm_fields(self):
        raw = {
            "command_type": "arm",
            "payload": {
                "joint_positions_rad": [0.0, -0.1, 0.2],
                "joint_names": ["j1", "j2", "j3"],
            },
            "metadata": {"operator": "demo"},
        }
        msg = command_dict_to_msg(raw)
        out = command_msg_to_dict(msg, default_source="ros2")

        self.assertEqual(out["command_type"], "arm")
        self.assertEqual(out["payload"]["joint_positions_rad"], [0.0, -0.1, 0.2])
        self.assertEqual(out["payload"]["joint_names"], ["j1", "j2", "j3"])
        self.assertEqual(out["metadata"]["operator"], "demo")


if __name__ == "__main__":
    unittest.main()
