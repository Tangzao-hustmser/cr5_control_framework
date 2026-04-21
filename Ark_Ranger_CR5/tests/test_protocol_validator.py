import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.protocol import PROTOCOL_VERSION
from src.r3.validator import RuntimeModel, validate_and_normalize


class TestProtocolValidator(unittest.TestCase):
    def setUp(self):
        self.runtime = RuntimeModel(
            arm_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            arm_joint_limits_rad=[(-3.14, 3.14), (-3.14, 3.14), (-2.86, 2.86), (-3.14, 3.14), (-3.14, 3.14), (-6.28, 6.28)],
            gripper_joint_name="finger_joint",
            gripper_limits_rad=(0.0, 0.6524),
            allowed_frames=["world"],
            workspace_limits_m={"x": (-1.0, 1.0), "y": (-1.0, 1.0), "z": (0.0, 1.2)},
        )

    def test_pose_command_valid(self):
        msg = {
            "protocol_version": PROTOCOL_VERSION,
            "command_id": "cmd-1",
            "timestamp_ms": 1710000000000,
            "source": "test",
            "command_type": "pose",
            "payload": {
                "frame": "world",
                "position_m": [0.1, 0.2, 0.3],
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "metadata": {},
        }
        result = validate_and_normalize(msg, self.runtime, source="test")
        self.assertTrue(result.ok)
        self.assertIsNotNone(result.command)
        self.assertEqual(result.command.command_type, "pose")
        self.assertEqual(len(result.command.pose_target_wxyz), 7)

    def test_unknown_field_rejected(self):
        msg = {
            "command_type": "arm",
            "payload": {"joint_positions_rad": [0, 0, 0, 0, 0, 0]},
            "illegal": 1,
        }
        result = validate_and_normalize(msg, self.runtime, source="test")
        self.assertFalse(result.ok)
        self.assertTrue(any(i.fault_code == "R3-E1102" for i in result.issues))

    def test_arm_dimension_mismatch(self):
        msg = {
            "command_type": "arm",
            "payload": {"joint_positions_rad": [0, 0, 0]},
        }
        result = validate_and_normalize(msg, self.runtime, source="test")
        self.assertFalse(result.ok)
        self.assertTrue(any(i.fault_code == "R3-E1201" for i in result.issues))

    def test_reject_legacy_message(self):
        # legacy schema: mode/target/gripper should be rejected
        msg = {"mode": "pose", "target": [0.4, 0.1, 0.5, 1, 0, 0, 0], "gripper": [0.0]}
        result = validate_and_normalize(msg, self.runtime, source="legacy")
        self.assertFalse(result.ok)
        self.assertTrue(any(i.fault_code == "R3-E1102" for i in result.issues))

    def test_same_payload_same_internal_shape(self):
        payload = {
            "command_type": "arm",
            "payload": {"joint_positions_rad": [0.0, -0.3, 0.8, 0.1, 0.2, -0.2]},
        }
        a = validate_and_normalize(payload, self.runtime, source="local_terminal")
        b = validate_and_normalize(payload, self.runtime, source="ros2")
        self.assertTrue(a.ok and b.ok)
        self.assertEqual(a.command.command_type, b.command.command_type)
        self.assertEqual(a.command.arm_target_rad, b.command.arm_target_rad)


if __name__ == "__main__":
    unittest.main()
