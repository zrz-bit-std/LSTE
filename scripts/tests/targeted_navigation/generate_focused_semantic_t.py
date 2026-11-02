#!/usr/bin/env python3
"""Generate a compact single-junction T world for Portal/WorkItem lifecycle validation."""
import importlib.util
from pathlib import Path

script = Path(__file__).with_name("generate_semantic_room_worlds.py")
spec = importlib.util.spec_from_file_location("semantic_rooms", script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

module.ARMS.update({
    "west": (-20.0, -12.0, -4.0, 4.0),
    "east": (12.0, 20.0, -4.0, 4.0),
    "north": (-4.0, 4.0, 12.0, 20.0),
})
module.CORRIDORS.update({
    "west": (-12.0, -1.5, -1.5, 1.5),
    "east": (1.5, 12.0, -1.5, 1.5),
    "north": (-1.5, 1.5, 1.5, 12.0),
})
module.SPAWN = [-8.0, 0.0, 0.0]
module.EXTRA_CORRIDORS = ["west"]
module.generate("semantic_t_junction_focused", [
    ("east", "office"), ("north", "meeting"),
])
