import os
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.asset_check import run_asset_consistency_check


class TestAssetChecker(unittest.TestCase):
    def test_asset_check_runs_and_writes_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            report_path = os.path.join(tmp, "asset_report.json")
            report = run_asset_consistency_check(
                project_root=ROOT,
                robot_config_path=os.path.join(ROOT, "configs", "robot_config.yaml"),
                kinematics_config_path=os.path.join(ROOT, "configs", "kinematics_config.yaml"),
                report_output_path=report_path,
            )
            self.assertTrue(os.path.exists(report_path))
            self.assertIn("summary", report)
            self.assertIn(report["summary"]["status"], ("NORMAL", "DEGRADED", "BLOCKED"))
            # For current checked-in assets, no critical mismatch is expected.
            self.assertEqual(report["summary"]["critical"], 0)


if __name__ == "__main__":
    unittest.main()
