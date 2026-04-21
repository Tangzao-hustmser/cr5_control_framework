import math
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.adapters import AdapterMessage, LocalInputAdapter


class TestLocalAdapterShortcut(unittest.TestCase):
    def _poll_single(self, text: str):
        adapter = LocalInputAdapter(show_help=False)
        adapter._queue.put(AdapterMessage(source="local_terminal", raw=text, received_ms=1))  # noqa: SLF001
        out = adapter.poll()
        self.assertEqual(len(out), 1)
        return out[0].raw

    def test_arm_shortcut_deg_to_rad(self):
        raw = self._poll_single("arm 0 -30 45 0 0 0")
        self.assertEqual(raw.get("command_type"), "arm")
        target = raw.get("payload", {}).get("joint_positions_rad", [])
        self.assertEqual(len(target), 6)
        self.assertAlmostEqual(target[0], 0.0, places=6)
        self.assertAlmostEqual(target[1], -math.pi / 6.0, places=6)
        self.assertAlmostEqual(target[2], math.pi / 4.0, places=6)

    def test_arm_shortcut_rad_passthrough(self):
        raw = self._poll_single("arm_rad 0 -0.5 1.0 0 0.2 -0.1")
        self.assertEqual(raw.get("command_type"), "arm")
        target = raw.get("payload", {}).get("joint_positions_rad", [])
        self.assertEqual(target, [0.0, -0.5, 1.0, 0.0, 0.2, -0.1])

    def test_arm_shortcut_non_numeric_rejected(self):
        raw = self._poll_single("arm 0 A 10 0 0 0")
        self.assertIn("__adapter_error__", raw)
        err = raw["__adapter_error__"]
        self.assertEqual(err.get("fault_code"), "R3-E1001")


if __name__ == "__main__":
    unittest.main()
