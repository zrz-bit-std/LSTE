"""Keep the legacy GP comparison node navigable as it evolves."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


class GpSubgoalModuleLayoutTest(unittest.TestCase):
    def test_constructor_only_composes_bootstrap_phases(self):
        source = (SCRIPTS / "gp_subgoals_sim_topo.py").read_text(encoding="utf-8")
        bootstrap = (SCRIPTS / "gp_frontier_bootstrap.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("class VSGPNavGlb(", source)
        self.assertIn("GpFrontierBootstrapMixin,", source)
        for method in (
            "_initialize_sensor_cache",
            "_setup_core_ros_interfaces",
            "_initialize_access_topology_state",
            "_setup_access_topology_ros_interfaces",
            "_load_gp_runtime_configuration",
            "_configure_frontier_log",
        ):
            self.assertIn("self.%s()" % method, source)
            self.assertIn("def %s(" % method, bootstrap)

    def test_cmake_installs_the_entrypoint_and_its_bootstrap_module(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/gp_subgoals_sim_topo.py", cmake)
        self.assertIn("scripts/gp_frontier_bootstrap.py", cmake)
        self.assertIn("scripts/gp_frontier_sparse_gp.py", cmake)

    def test_sparse_model_has_one_node_local_implementation(self):
        source = (SCRIPTS / "gp_subgoals_sim_topo.py").read_text(encoding="utf-8")
        model = (SCRIPTS / "gp_frontier_sparse_gp.py").read_text(encoding="utf-8")
        model_mixin = (SCRIPTS / "gp_frontier_model.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("from gp_frontier_sparse_gp import SGP2D", model_mixin)
        self.assertNotIn("class SGP2D:", source)
        self.assertIn("class SGP2D:", model)

    def test_entrypoint_composes_extracted_algorithm_responsibilities(self):
        source = (SCRIPTS / "gp_subgoals_sim_topo.py").read_text(encoding="utf-8")
        for filename, mixin, method in (
            ("gp_frontier_callbacks_config.py", "GpFrontierCallbacksAndConfigurationMixin", "lste_state_cb"),
            ("gp_frontier_junction.py", "GpFrontierJunctionMixin", "cluster_frontiers"),
            ("gp_frontier_lifecycle.py", "GpFrontierLifecycleMixin", "dump_access_topo"),
            ("gp_frontier_model.py", "GpFrontierModelMixin", "gp_nav_fit"),
            ("gp_frontier_geometry_candidates.py", "GpFrontierGeometryAndCandidatesMixin", "gp_nav_pkup_nav_pt"),
            ("gp_frontier_publication.py", "GpFrontierPublicationMixin", "publish_frontiers"),
            ("gp_frontier_visualization.py", "GpFrontierVisualizationMixin", "sph_pcl_cb"),
            ("gp_frontier_runtime.py", "GpFrontierRuntimeMixin", "step"),
        ):
            module = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn(mixin, source)
            self.assertIn("class %s" % mixin, module)
            self.assertIn("def %s(" % method, module)
            self.assertNotIn("    def %s(" % method, source)

        topology = (SCRIPTS / "gp_frontier_topology.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("GpFrontierTopologyMixin", source)
        self.assertIn("class GpFrontierTopologyMixin", topology)
        for filename, mixin, method in (
            ("gp_frontier_anchor_graph.py", "GpFrontierAnchorGraphMixin", "ensure_anchor"),
            ("gp_frontier_junction_logging.py", "GpFrontierJunctionLoggingMixin", "_maybe_init_junction_log"),
            ("gp_frontier_backtrack.py", "GpFrontierBacktrackMixin", "enter_backtrack"),
            ("gp_frontier_forced_branch.py", "GpFrontierForcedBranchMixin", "update_forced_heading_progress"),
        ):
            module = (SCRIPTS / filename).read_text(encoding="utf-8")
            self.assertIn(mixin, topology)
            self.assertIn("class %s" % mixin, module)
            self.assertIn("def %s(" % method, module)

    def test_entrypoint_constructs_one_node_before_entering_the_loop(self):
        source = (SCRIPTS / "gp_subgoals_sim_topo.py").read_text(encoding="utf-8")
        self.assertIn("node = VSGPNavGlb()", source)
        self.assertIn("node.step()", source)
        self.assertNotIn("VSGPNavGlb().step()", source)
