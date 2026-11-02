"""The 3-minute stall rule, replayed against recorded runs.

User rule: a scenario may use at most 40 minutes, and any uninterrupted loss of
progress longer than 3 minutes counts as failure and must end the run.

Measured history: the longest *normal* pause in a successful L run was 69 s,
while pathological stalls lasted 568-2282 s.  A 180 s threshold therefore sits
at 2.6x the normal worst case.

This test replays the rule against

* the successful L baseline (20261028_050348) -- must never trip, and
* the stalled T run (20261031_195131) -- must trip about 180 s into the stall,
  which is roughly 2000 s earlier than waiting for the window to expire.
"""

from pathlib import Path
import os
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts" / "tests" / "targeted_navigation"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import l_stall_watchdog as watchdog  # noqa: E402


LOGS = Path(os.environ.get(
    "LSTE_TJUNCTION_LOGS",
    "/home/zrz/work/lste_ws/runtime/targeted_navigation/t_junction_small/logs",
))

SUCCESS = LOGS / "20261028_050348"
STALLED = LOGS / "20261031_195131"

THRESHOLD = 180.0


def load(run_dir):
    log = run_dir / ("%s_navigation_metrics.log" % run_dir.name)
    samples = []
    with open(log, errors="replace") as handle:
        for line in handle:
            watchdog.parse_samples([line], samples)
    return samples


def first_stall(samples, threshold=THRESHOLD):
    for index in range(len(samples)):
        latest = samples[index][0]
        window = [s for s in samples[:index + 1] if s[0] >= latest - threshold]
        if len(window) < 2:
            continue
        if watchdog.stalled_seconds(window, threshold) >= threshold:
            return latest
    return None


class SyntheticRuleTest(unittest.TestCase):
    def test_stationary_robot_trips(self):
        samples = [(float(t), 1.0, 2.0) for t in range(0, 400, 2)]
        self.assertGreaterEqual(watchdog.stalled_seconds(samples, 180.0), 180.0)

    def test_moving_robot_does_not_trip(self):
        samples = [(float(t), 0.05 * t, 0.0) for t in range(0, 400, 2)]
        self.assertEqual(watchdog.stalled_seconds(samples, 180.0), 0.0)

    def test_short_pause_does_not_trip(self):
        samples = [(float(t), 0.0, 0.0) for t in range(0, 70, 2)]
        samples += [(70.0 + t, 0.05 * t, 0.0) for t in range(2, 100, 2)]
        self.assertEqual(watchdog.stalled_seconds(samples, 180.0), 0.0)


@unittest.skipUnless(SUCCESS.is_dir() and STALLED.is_dir(),
                     "recorded runs not present")
class RecordedRunReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.success = load(SUCCESS)
        cls.stalled = load(STALLED)

    def test_successful_run_never_trips(self):
        self.assertIsNone(first_stall(self.success))

    def test_stalled_run_trips_early(self):
        tripped = first_stall(self.stalled)
        self.assertIsNotNone(tripped)
        # The robot stopped around sim 216; a 180 s rule must fire near 396 s,
        # far before the 2482 s the run actually consumed.
        self.assertGreaterEqual(tripped, 350.0)
        self.assertLess(tripped, 500.0)

    def test_rule_saves_most_of_the_wasted_window(self):
        tripped = first_stall(self.stalled)
        end = self.stalled[-1][0]
        self.assertGreater(end - tripped, 1500.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
