#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compatibility facade for GoalManager ROS callbacks.

Input-message handling and TEB action outcomes have separate state-machine
lifecycles. Keep this facade so existing node composition and external imports
remain stable while each responsibility stays in a focused module.
"""

from goal_manager_input_callbacks import GoalManagerInputCallbacksMixin
from goal_manager_teb_callbacks import GoalManagerTebCallbacksMixin


class GoalManagerRosCallbacksMixin(
    GoalManagerInputCallbacksMixin,
    GoalManagerTebCallbacksMixin,
):
    """Backward-compatible composition of GoalManager callback groups."""

    pass
