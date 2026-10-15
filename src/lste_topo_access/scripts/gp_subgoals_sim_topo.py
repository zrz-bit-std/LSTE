#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compose the legacy GP-frontier comparison node.

The modern online-SLAM explorer is the normal LSTE path.  This node remains
available for GP/Access-Topo comparisons, with ROS wiring, GP inference,
topology, and visualization kept in focused sibling modules.
"""

import faulthandler
import os
import warnings

import rospy


faulthandler.enable()
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
warnings.filterwarnings("ignore")

from gp_frontier_bootstrap import GpFrontierBootstrapMixin
from gp_frontier_callbacks_config import GpFrontierCallbacksAndConfigurationMixin
from gp_frontier_geometry_candidates import GpFrontierGeometryAndCandidatesMixin
from gp_frontier_junction import GpFrontierJunctionMixin
from gp_frontier_lifecycle import GpFrontierLifecycleMixin
from gp_frontier_model import GpFrontierModelMixin
from gp_frontier_publication import GpFrontierPublicationMixin
from gp_frontier_runtime import GpFrontierRuntimeMixin
from gp_frontier_topology import GpFrontierTopologyMixin
from gp_frontier_visualization import GpFrontierVisualizationMixin


class VSGPNavGlb(
    GpFrontierRuntimeMixin,
    GpFrontierVisualizationMixin,
    GpFrontierPublicationMixin,
    GpFrontierGeometryAndCandidatesMixin,
    GpFrontierModelMixin,
    GpFrontierLifecycleMixin,
    GpFrontierJunctionMixin,
    GpFrontierTopologyMixin,
    GpFrontierCallbacksAndConfigurationMixin,
    GpFrontierBootstrapMixin,
):
    """Assemble the legacy GP frontier node without mixing responsibilities."""

    def __init__(self):
        rospy.init_node("gp_subgoal")
        print("##############################################")
        print("              Initialize gp_subgoal           ")
        print("##############################################")
        self._initialize_sensor_cache()
        self._setup_core_ros_interfaces()
        self._initialize_access_topology_state()
        self._setup_access_topology_ros_interfaces()
        self._load_gp_runtime_configuration()
        self._configure_frontier_log()


if __name__ == "__main__":
    node = VSGPNavGlb()
    try:
        node.step()
    except rospy.ROSInterruptException:
        pass
