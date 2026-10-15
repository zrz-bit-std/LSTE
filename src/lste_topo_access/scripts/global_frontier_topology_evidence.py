"""Stable evidence comparisons between successive topology snapshots."""

def copy_component_evidence(component):
    """Copy compact, serializable component evidence into region memory."""
    if component is None:
        return None
    return {
        "epoch": int(component["epoch"]),
        "label": int(component["label"]),
        "cells": int(component["cells"]),
        "center_x": float(component["center_x"]),
        "center_y": float(component["center_y"]),
        "min_x": float(component["min_x"]),
        "max_x": float(component["max_x"]),
        "min_y": float(component["min_y"]),
        "max_y": float(component["max_y"]),
    }


def same_topology_component(first, second):
    """Return whether two descriptors name one current-map free-space core.

    Topology labels are intentionally only meaningful inside one map epoch.
    This predicate is therefore deliberately stricter than the region-memory
    signature bridge: a target-room claim must never cross a doorway merely
    because two old component centroids happen to be close.
    """
    if first is None or second is None:
        return False
    try:
        return (
            int(first["epoch"]) == int(second["epoch"])
            and int(first["label"]) == int(second["label"])
        )
    except (KeyError, TypeError, ValueError):
        return False



