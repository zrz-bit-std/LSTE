"""Keep navigation telemetry small enough to navigate by responsibility."""

from pathlib import Path
import importlib.util
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


class NavigationMetricsModuleLayoutTest(unittest.TestCase):
    def test_periodic_snapshot_keeps_compact_route_identity(self):
        source = (SCRIPTS / "navigation_metrics_snapshot.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("def _route_identity_snapshot(self):", source)
        self.assertIn('"route_identity": self._route_identity_snapshot()', source)

        spec = importlib.util.spec_from_file_location(
            "navigation_metrics_snapshot_test_module",
            SCRIPTS / "navigation_metrics_snapshot.py",
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)

        class Harness(module.NavigationMetricsSnapshotMixin):
            def _failure_route_context_locked(self):
                return {
                    "route": {
                        "route_id": 12,
                        "route_kind": "portal_transition",
                        "graph_action": "cross_portal",
                        "place_id": 3,
                        "portal_id": 7,
                    },
                    "contexts": {"frontier": {"payload": {"large": "ignored"}}},
                }

        self.assertEqual(
            Harness()._route_identity_snapshot(),
            {
                "route_id": 12,
                "route_kind": "portal_transition",
                "graph_action": "cross_portal",
                "place_id": 3,
                "portal_id": 7,
            },
        )

    def test_command_telemetry_separates_events_from_final_command_accounting(self):
        facade = (SCRIPTS / "navigation_metrics_command_quality.py").read_text(
            encoding="utf-8"
        )
        events = (SCRIPTS / "navigation_metrics_command_events.py").read_text(
            encoding="utf-8"
        )
        stream = (SCRIPTS / "navigation_metrics_command_stream.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("NavigationMetricsCommandEventsMixin", facade)
        self.assertIn("NavigationMetricsCommandStreamMixin", facade)
        self.assertLessEqual(len(facade.splitlines()), 30)
        self.assertIn("def _command_discontinuity_reason_locked(", events)
        self.assertIn("def on_teb_cmd(", events)
        self.assertIn("def on_cmd(", stream)
        self.assertIn("def on_cmd_vel_mux_status(", stream)

    def test_cmake_installs_each_command_telemetry_module(self):
        cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text(encoding="utf-8")
        for filename in (
            "scripts/navigation_metrics_command_events.py",
            "scripts/navigation_metrics_command_quality.py",
            "scripts/navigation_metrics_command_stream.py",
        ):
            self.assertIn(filename, cmake)

    def test_failure_evidence_module_is_installed_with_the_metrics_siblings(self):
        source = (SCRIPTS / "lste_navigation_metrics.py").read_text(
            encoding="utf-8"
        )
        cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("NavigationMetricsFailureEvidenceMixin", source)
        self.assertIn("scripts/navigation_metrics_failure_evidence.py", cmake)

    def test_timestamp_remap_is_string_safe_before_rospy_init(self):
        """A date-like timestamp must not become an oversized XML-RPC int."""
        from rospy.client import load_command_line_node_params
        from unittest import mock

        import navigation_metrics_setup

        from navigation_metrics_setup import (
            _private_remap_value,
            _remove_private_remap,
        )

        argv = [
            "lste_navigation_metrics.py",
            "_run_timestamp:=20260908_072000",
            "_run_directory:=/tmp/lste-run/20260908_072000",
            "_retention_days:=15",
        ]
        timestamp = _private_remap_value(argv, "run_timestamp")
        init_argv = _remove_private_remap(argv, "run_timestamp")

        self.assertEqual(timestamp, "20260908_072000")
        self.assertIsInstance(timestamp, str)
        self.assertNotIn("_run_timestamp:=20260908_072000", init_argv)
        # This is the same parser used by rospy.init_node.  It can still
        # decode the remaining arguments, and no oversized timestamp integer
        # reaches the XML-RPC parameter upload.
        params = load_command_line_node_params(init_argv)
        self.assertEqual(
            params,
            {
                "run_directory": "/tmp/lste-run/20260908_072000",
                "retention_days": 15,
            },
        )

        with mock.patch.object(navigation_metrics_setup.rospy, "init_node") as init_node:
            with mock.patch.object(navigation_metrics_setup.rospy, "set_param") as set_param:
                navigation_metrics_setup._init_metrics_node(argv)

        init_node.assert_called_once_with(
            "lste_navigation_metrics",
            argv=[
                "lste_navigation_metrics.py",
                "_run_directory:=/tmp/lste-run/20260908_072000",
                "_retention_days:=15",
            ],
        )
        set_param.assert_called_once_with(
            "~run_timestamp", "20260908_072000"
        )


if __name__ == "__main__":
    unittest.main()
