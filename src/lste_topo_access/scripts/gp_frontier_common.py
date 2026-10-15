"""Small dependency-free helpers shared by extracted GP frontier modules."""

import os
from pathlib import Path

import numpy as np
import yaml


def resolve_topo_path(rel_path: str) -> str:
    """Resolve a topology path from the workspace, package, or local source."""
    if os.path.isabs(rel_path) and rel_path:
        return rel_path
    workspace = os.environ.get("LSTE_WS")
    if workspace:
        base = os.path.join(workspace, "src", "lste_topo_access")
        return os.path.join(base, rel_path) if rel_path else os.path.join(base, "topo_tree", "tree")
    try:
        import rospkg

        package = rospkg.RosPack().get_path("lste_topo_access")
        return os.path.join(package, rel_path) if rel_path else os.path.join(package, "topo_tree", "tree")
    except Exception:
        package = Path(__file__).resolve().parents[1]
        return str(package / rel_path) if rel_path else str(package / "topo_tree" / "tree")


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi]``."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def angle_diff(first: float, second: float) -> float:
    """Return the smallest absolute angular distance."""
    return abs(wrap_angle(first - second))


def safe_load_yaml(path):
    """Load a mapping from YAML, returning an empty mapping on failure."""
    try:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}
