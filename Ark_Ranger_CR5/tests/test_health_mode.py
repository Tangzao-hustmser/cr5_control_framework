import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.health import run_startup_health_check


class TestHealthMode(unittest.TestCase):
    def _robot_cfg(self):
        return {
            "robot": {
                "name": "demo",
                "prim_path": "/World/Demo",
                "usd_path": "models/demo.usd",
                "urdf_path": "models/demo.urdf",
            },
            "action_space": {
                "arm": {"joints": ["j1", "j2"]},
                "gripper": {"joints": ["g1"]},
            },
        }

    def test_ros2_only_rejects_non_ros2_input(self):
        runtime_cfg = {
            "mode": "ros2_only",
            "input_source": "local_terminal",
            "compat": {"enable_adapters": True},
        }
        report = run_startup_health_check(self._robot_cfg(), runtime_cfg, asset_report=None)
        issues = [issue.code for issue in report.issues]
        self.assertIn("R3-E1701", issues)
        self.assertEqual(report.details.get("mode"), "ros2_only")


if __name__ == "__main__":
    unittest.main()
