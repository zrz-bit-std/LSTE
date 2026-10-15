"""Regression tests for semantic target-track identity."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from target_track_identity import (  # noqa: E402
    TargetTrackIdentity,
    canonical_label,
    labels_compatible,
)
from goal_manager_target_utils import GoalManagerTargetUtilsMixin  # noqa: E402


class TargetTrackIdentityTest(unittest.TestCase):
    def test_synonym_keeps_the_same_track(self):
        self.assertTrue(labels_compatible("yellow cup", "yellow mug"))
        self.assertEqual(canonical_label("yellow mug"), canonical_label("yellow cup"))

    def test_conflicting_attribute_is_a_new_identity(self):
        self.assertFalse(labels_compatible("yellow cup", "blue cup"))

    def test_different_object_is_a_new_identity(self):
        self.assertFalse(labels_compatible("yellow cup", "yellow monitor"))

    def test_missing_optional_attribute_is_compatible(self):
        self.assertTrue(labels_compatible("yellow cup", "cup"))

    def test_identity_object_is_immutable_and_queryable(self):
        identity = TargetTrackIdentity.from_label("yellow mug")
        self.assertEqual(identity.canonical, "cup yellow")
        self.assertTrue(identity.accepts("yellow cup"))

    def test_locked_track_does_not_fallback_to_highest_scoring_other_object(self):
        class Manager(GoalManagerTargetUtilsMixin):
            target_track_label = "yellow cup"

        target = SimpleNamespace(label="yellow cup", score=0.2)
        distractor = SimpleNamespace(label="blue cup", score=0.99)
        message = SimpleNamespace(target_dets=[distractor, target])
        self.assertIs(Manager().target_detection_for_track(message), target)
        self.assertIs(
            Manager().target_detection_for_track(
                SimpleNamespace(target_dets=[distractor])
            ),
            None,
        )


if __name__ == "__main__":
    unittest.main()
