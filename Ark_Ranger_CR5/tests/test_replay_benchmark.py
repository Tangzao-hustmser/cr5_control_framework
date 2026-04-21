import json
import os
import sys
import tempfile
import unittest
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.benchmark import compute_metrics_from_logs, load_scenarios
from src.r3.replay import ReplayEngine


class TestReplayAndBenchmark(unittest.TestCase):
    def _write_jsonl(self, path, rows):
        with open(path, "w", encoding="utf-8") as fh:
            for row in rows:
                fh.write(json.dumps(row) + "\n")

    def test_replay_and_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            self._write_jsonl(
                os.path.join(tmp, "validation.jsonl"),
                [
                    {"command_id": "c1", "ok": True, "normalized": {"command_type": "arm"}},
                    {"command_id": "c2", "ok": False, "normalized": None},
                ],
            )
            self._write_jsonl(
                os.path.join(tmp, "state.jsonl"),
                [
                    {
                        "command_id": "c1",
                        "monotonic_s": 1.0,
                        "state": {
                            "target_arm_rad": [0, 0, 0],
                            "current_arm_rad": [0.1, 0.0, 0.0],
                        },
                    },
                    {
                        "command_id": "c1",
                        "monotonic_s": 2.0,
                        "state": {
                            "target_arm_rad": [0, 0, 0],
                            "current_arm_rad": [0.01, 0.0, 0.0],
                        },
                    },
                ],
            )
            self._write_jsonl(os.path.join(tmp, "events.jsonl"), [{"event": {"fault_code": "R3-E1501"}}])
            self._write_jsonl(os.path.join(tmp, "safety.jsonl"), [{"decision": "clamp"}])

            engine = ReplayEngine(tmp)
            replay_result = engine.replay(headless=True)
            self.assertEqual(replay_result.replayed_commands, 1)
            self.assertEqual(replay_result.dropped_commands, 1)

            metrics = compute_metrics_from_logs(tmp)
            self.assertGreaterEqual(metrics.max_error, metrics.mean_error)
            self.assertGreater(metrics.ik_failure_rate, 0.0)
            self.assertGreater(metrics.safety_intercept_rate, 0.0)

    def test_load_scenarios(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "scenarios.yaml")
            with open(path, "w", encoding="utf-8") as fh:
                yaml.safe_dump({"scenarios": [{"id": "s1", "command_sequence": []}]}, fh)
            data = load_scenarios(path)
            self.assertEqual(len(data["scenarios"]), 1)


if __name__ == "__main__":
    unittest.main()
