"""Optional per-frame junction-angle diagnostic log."""

import json
import os

import rospy

from gp_frontier_common import wrap_angle


class GpFrontierJunctionLoggingMixin:
    """Persist one junction session only when angle logging is enabled."""

    def _maybe_init_junction_log(self, junction: dict):
        if not getattr(self, "junction_angle_log", False) or junction.get("log_inited"):
            return
        try:
            os.makedirs(self.junction_angle_dir, exist_ok=True)
        except Exception:
            pass
        self.junction_angle_counter += 1
        filename = "junction_%d_%s_%d.json" % (
            int(rospy.Time.now().to_sec()),
            junction.get("node_id", "x"),
            self.junction_angle_counter,
        )
        junction["log_path"] = os.path.join(self.junction_angle_dir, filename)
        junction["log_frames"] = []
        junction["log_inited"] = True

    def _append_junction_log(self, junction: dict, stamp: float, thetas_rel, pose):
        if (
            not getattr(self, "junction_angle_log", False)
            or not junction.get("log_inited")
        ):
            return
        try:
            frame = {
                "stamp": float(stamp),
                "thetas_rel": [float(theta) for theta in thetas_rel],
            }
            if pose is not None and thetas_rel is not None and len(thetas_rel) > 0:
                frame["headings_world"] = [
                    float(wrap_angle(pose.theta + theta)) for theta in thetas_rel
                ]
            junction.setdefault("log_frames", []).append(frame)
        except Exception:
            return

    def _finalize_junction_log(self, junction: dict, status: str = "finalized"):
        if (
            not getattr(self, "junction_angle_log", False)
            or not junction.get("log_inited")
        ):
            return
        data = {
            "node_id": junction.get("node_id"),
            "created": junction.get("created"),
            "anchor_xy": junction.get("anchor_xy"),
            "status": status,
            "chosen_id": junction.get("chosen_id"),
            "candidates": junction.get("candidates"),
            "frames": junction.get("log_frames", []),
        }
        try:
            os.makedirs(self.junction_angle_dir, exist_ok=True)
            path = junction.get("log_path")
            if path:
                with open(path, "w", encoding="utf-8") as handle:
                    json.dump(data, handle, ensure_ascii=False, indent=2)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "save junction angle log failed: %s", exc)
