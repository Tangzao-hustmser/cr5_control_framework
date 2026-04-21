import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.protocol import NormalizedCommand
from src.r3.safety import RobotStateSnapshot, SafetyFilterV1
from src.r3.validator import RuntimeModel


class TestSafetyFilter(unittest.TestCase):
    def setUp(self):
        runtime = RuntimeModel(
            arm_joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
            arm_joint_limits_rad=[(-1, 1)] * 6,
            gripper_joint_name="finger",
            gripper_limits_rad=(0.0, 0.6),
            allowed_frames=["world"],
            workspace_limits_m={"x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (0.0, 1.0)},
        )
        self.filter = SafetyFilterV1(runtime, max_pose_deviation_m=0.3, max_joint_deviation_rad=0.7)
        self.state = RobotStateSnapshot(
            joint_positions_rad=[0.0] * 6,
            ee_pose_wxyz=[0, 0, 0, 1, 0, 0, 0],
            gripper_position_rad=0.0,
        )

    def test_pass_arm(self):
        cmd = NormalizedCommand(
            command_id="c1",
            protocol_version="r3.v1",
            timestamp_ms=1,
            source="test",
            command_type="arm",
            arm_target_rad=[0.1] * 6,
        )
        decision = self.filter.apply(cmd, self.state)
        self.assertEqual(decision.decision, "pass")

    def test_clamp_arm(self):
        cmd = NormalizedCommand(
            command_id="c2",
            protocol_version="r3.v1",
            timestamp_ms=1,
            source="test",
            command_type="arm",
            arm_target_rad=[1.2, 0, 0, 0, 0, 0],
        )
        decision = self.filter.apply(cmd, self.state)
        self.assertEqual(decision.decision, "clamp")
        self.assertIn("R3-E1402", decision.reason_codes)

    def test_block_arm_far_outside(self):
        cmd = NormalizedCommand(
            command_id="c3",
            protocol_version="r3.v1",
            timestamp_ms=1,
            source="test",
            command_type="arm",
            arm_target_rad=[2.5, 0, 0, 0, 0, 0],
        )
        decision = self.filter.apply(cmd, self.state)
        self.assertEqual(decision.decision, "block")
        self.assertIn("R3-E1401", decision.reason_codes)


if __name__ == "__main__":
    unittest.main()
