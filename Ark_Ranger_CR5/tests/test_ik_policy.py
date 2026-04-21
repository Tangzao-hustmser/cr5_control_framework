import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.r3.ik_policy import IKFailurePolicy


class TestIKPolicy(unittest.TestCase):
    def test_retry_then_fallback(self):
        policy = IKFailurePolicy(max_retries=2)
        d1 = policy.on_failure([0, 0, 0, 0, 0, 0])
        d2 = policy.on_failure([0, 0, 0, 0, 0, 0])
        d3 = policy.on_failure([0, 0, 0, 0, 0, 0])

        self.assertEqual(d1.reason_code, "R3-E1501")
        self.assertEqual(d2.reason_code, "R3-E1501")
        self.assertEqual(d3.reason_code, "R3-E1502")
        self.assertEqual(d3.state, "FALLBACK_LAST_VALID")

    def test_success_resets_retry_counter(self):
        policy = IKFailurePolicy(max_retries=1)
        policy.on_failure([0, 0, 0, 0, 0, 0])
        d = policy.on_success([1, 1, 1, 1, 1, 1])
        self.assertEqual(d.retry_count, 0)
        self.assertEqual(policy.state, "NORMAL")


if __name__ == "__main__":
    unittest.main()
