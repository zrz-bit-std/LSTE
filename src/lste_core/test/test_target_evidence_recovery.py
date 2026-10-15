"""Evidence-preserving target post-processing tests."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from utils.target_evidence import preserve_raw_target_evidence


class TargetEvidenceRecoveryTest(unittest.TestCase):
    def test_raw_candidate_survives_a_hard_postprocess_rejection(self):
        raw = ([0.5, 0.5, 0.03, 0.03],)
        result = preserve_raw_target_evidence(
            raw, (0.42,), ("yellow cup",), (), (), (),
        )
        self.assertEqual(result[0], raw)
        self.assertTrue(result[3])
        self.assertEqual(
            result[4], "postprocess_removed_all_target_candidates"
        )

    def test_accepted_evidence_is_not_duplicated(self):
        accepted = ([0.5, 0.5, 0.03, 0.03],)
        result = preserve_raw_target_evidence(
            accepted, (0.42,), ("yellow cup",), accepted, (0.42,), ("yellow cup",)
        )
        self.assertEqual(result[0], accepted)
        self.assertFalse(result[3])
        self.assertEqual(result[4], "accepted")

    def test_empty_model_result_remains_empty(self):
        result = preserve_raw_target_evidence((), (), (), (), (), ())
        self.assertEqual(result[:3], ((), (), ()))
        self.assertFalse(result[3])


if __name__ == "__main__":
    unittest.main()
