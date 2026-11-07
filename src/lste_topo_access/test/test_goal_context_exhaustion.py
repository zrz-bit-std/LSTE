"""Regression tests for bounded context-pair exhaustion."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_context_exhaustion import ContextPairExhaustion  # noqa: E402


class ContextPairExhaustionTest(unittest.TestCase):
    def test_pair_is_paused_then_rearmed(self):
        memory = ContextPairExhaustion(cooldown=120.0)
        key = memory.key("plant", "desk", "chair")
        memory.mark(key, now=10.0)
        self.assertTrue(memory.is_exhausted(key, 20.0))
        self.assertFalse(memory.is_exhausted(key, 131.0))
        self.assertFalse(memory.is_exhausted(key, 132.0))

    def test_pair_identity_is_task_and_label_scoped(self):
        memory = ContextPairExhaustion()
        plant = memory.key("plant", "desk", "chair")
        laptop = memory.key("laptop", "desk", "chair")
        other = memory.key("plant", "monitor", "chair")
        memory.mark(plant, now=0.0)
        self.assertTrue(memory.is_exhausted(plant, 1.0))
        self.assertFalse(memory.is_exhausted(laptop, 1.0))
        self.assertFalse(memory.is_exhausted(other, 1.0))

    def test_pair_order_is_irrelevant(self):
        memory = ContextPairExhaustion()
        first = memory.key("plant", "desk", "chair")
        second = memory.key("plant", "chair", "desk")
        self.assertEqual(first, second)
        memory.mark(first, now=0.0)
        self.assertTrue(memory.is_exhausted(second, 1.0))

    def test_target_evidence_clears_the_pair(self):
        memory = ContextPairExhaustion()
        key = memory.key("plant", "desk", "chair")
        memory.mark(key, now=0.0)
        self.assertTrue(memory.is_exhausted(key, 5.0))
        memory.clear(key)
        self.assertFalse(memory.is_exhausted(key, 5.0))

    def test_mark_counts_and_prune(self):
        memory = ContextPairExhaustion(cooldown=10.0)
        key = memory.key("plant", "desk", "chair")
        memory.mark(key, now=0.0)
        memory.mark(key, now=1.0, cooldown=20.0)
        self.assertTrue(memory.is_exhausted(key, 20.0))
        self.assertEqual(memory.prune(30.0), 1)
        self.assertFalse(memory.is_exhausted(key, 31.0))

    def test_limit_drops_oldest_record(self):
        memory = ContextPairExhaustion(cooldown=100.0, limit=2)
        first = memory.key("t", "a", "b")
        second = memory.key("t", "c", "d")
        third = memory.key("t", "e", "f")
        memory.mark(first, now=0.0)
        memory.mark(second, now=1.0)
        memory.mark(third, now=2.0)
        self.assertFalse(memory.is_exhausted(first, 3.0))
        self.assertTrue(memory.is_exhausted(second, 3.0))
        self.assertTrue(memory.is_exhausted(third, 3.0))


if __name__ == "__main__":
    unittest.main()
