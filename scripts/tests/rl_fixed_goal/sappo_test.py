#!/usr/bin/env python3
"""Stable SA-PPO fixed-goal entrypoint.

The controller implementation is split by responsibility beside this file:
the ROS/MPI runtime, shared lidar primitives, and each safety guard.  Keep
this file as the launch target used by existing lifecycle scripts.
"""

from sappo_runner import main


if __name__ == "__main__":
    main()
