"""Guard the TEB turn-supervisor module boundaries."""

import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


class TebTurnSupervisorModuleLayoutTest(unittest.TestCase):
    def test_entrypoint_only_composes_execution_modules(self):
        source = (SCRIPTS / "lste_teb_turn_supervisor.py").read_text(
            encoding="utf-8"
        )
        for mixin in (
            "TebTurnSupervisorCallbacksMixin",
            "TebTurnSupervisorControlMixin",
            "TebTurnSupervisorRoutesMixin",
            "TebTurnSupervisorTurnLifecycleMixin",
        ):
            self.assertIn(mixin, source)
        self.assertNotIn("    def on_timer(", source)
        self.assertNotIn("    def on_bridge_status(", source)
        self.assertNotIn("    def _activate_turn_locked(", source)

    def test_each_execution_concern_has_a_small_module(self):
        expected = {
            "teb_turn_supervisor_callbacks.py": "class TebTurnSupervisorCallbacksMixin",
            "teb_turn_supervisor_control.py": "class TebTurnSupervisorControlMixin",
            "teb_turn_supervisor_parameters.py": "def configure_turn_supervisor_parameters",
            "teb_turn_supervisor_routes.py": "class TebTurnSupervisorRoutesMixin",
            "teb_turn_supervisor_state.py": "def initialize_turn_supervisor_state",
            "teb_turn_supervisor_turn_lifecycle.py": "class TebTurnSupervisorTurnLifecycleMixin",
        }
        for filename, marker in expected.items():
            source = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn(marker, source)

    def test_install_rules_include_runtime_modules(self):
        cmake = (SCRIPTS.parent / "CMakeLists.txt").read_text(encoding="utf-8")
        for filename in (
            "teb_turn_supervisor_callbacks.py",
            "teb_turn_supervisor_control.py",
            "teb_turn_supervisor_contract.py",
            "teb_turn_supervisor_parameters.py",
            "teb_turn_supervisor_ros.py",
            "teb_turn_supervisor_routes.py",
            "teb_turn_supervisor_state.py",
            "teb_turn_supervisor_turn_lifecycle.py",
        ):
            self.assertIn("scripts/%s" % filename, cmake)


if __name__ == "__main__":
    unittest.main()
