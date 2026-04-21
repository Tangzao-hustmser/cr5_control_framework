import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.timeline import TimelineIndexer


class TestTimelineAlignment(unittest.TestCase):
    def test_timeline_index_and_event_mapping(self):
        with tempfile.TemporaryDirectory() as tmp:
            indexer = TimelineIndexer(tmp)
            state_idx = indexer.register_state_snapshot("state-1", 1000, {"value": 1})
            video_idx = indexer.register_video_frame("frame-1", 1001, {"frame": 1})
            mapping = indexer.link_event("evt-1002-000001-cmd-1", 1002, "cmd-1", "")

            self.assertEqual(state_idx, 0)
            self.assertEqual(video_idx, 1)
            self.assertIn("timeline_index", mapping)
            self.assertGreaterEqual(mapping["timeline_index"], 2)
            self.assertEqual(mapping["nearest_state"]["timeline_index"], state_idx)
            self.assertEqual(mapping["nearest_video"]["timeline_index"], video_idx)

            with open(os.path.join(tmp, "timeline_index.jsonl"), "r", encoding="utf-8") as fh:
                rows = [json.loads(line) for line in fh if line.strip()]

            self.assertEqual(len(rows), 3)
            self.assertEqual([row["timeline_index"] for row in rows], [0, 1, 2])
            self.assertEqual(rows[-1]["kind"], "event_index")
            self.assertEqual(rows[-1]["ref_id"], "evt-1002-000001-cmd-1")


if __name__ == "__main__":
    unittest.main()
