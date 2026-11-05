"""Junction observation tree and stable WeDetect room scoring.

The tree is deliberately independent of ROS and motion control.  The GP tree
owns physical junction/portal identities; this layer records the temporary
room observations collected while authenticating each portal.
"""
import math


class JunctionObservationTree:
    def __init__(self, stable_window_ratio=0.6, negative_score=-1.0,
                 semantic_scorer=None):
        self.stable_window_ratio = min(1.0, max(0.25, float(stable_window_ratio)))
        self.negative_score = float(negative_score)
        # Optional pure callable: labels -> score | None.  ``None`` keeps the
        # legacy mean detector confidence, so ROS-free tests and baselines
        # without a semantic task remain unchanged.
        self.semantic_scorer = semantic_scorer
        self.junctions = {}

    def begin(self, junction_id, branch_id, relative_heading):
        junction_id = int(junction_id)
        branch_id = int(branch_id)
        junction = self.junctions.setdefault(junction_id, {
            "junction_id": junction_id, "phase": "watching", "room_ids": [],
        })
        room = junction.setdefault("rooms", {}).get(branch_id)
        if room is None:
            room = {
                "branch_id": branch_id,
                "relative_heading": float(relative_heading),
                "invalidated": False,
                "exploration_suppressed": False,
                "semantic_score": None,
                "semantic_score_source": None,
                "detector_confidence_score": None,
                "priority": None,
                "place_id": None,
                "work_item_ids": [],
                "state": "observing",
                "frames": [],
                "labels": [],
                "stable_frame_count": 0,
            }
            junction["rooms"][branch_id] = room
            junction["room_ids"].append(branch_id)
        return room

    def prune_branch(self, junction_id, branch_id, *, invalidated=False):
        """Retain lineage for a deduplicated GP branch without watching it."""
        junction = self.junctions.get(int(junction_id))
        room = None if junction is None else junction.get("rooms", {}).get(int(branch_id))
        if room is None:
            return None
        room["state"] = "pruned"
        room["exploration_suppressed"] = True
        room["invalidated"] = bool(invalidated)
        room["semantic_score"] = self.negative_score
        room["semantic_score_source"] = "pruned"
        room["priority"] = None
        room["frames"] = []
        room["labels"] = []
        room["stable_frame_count"] = 0
        return room

    def observe(self, junction_id, branch_id, detections, stamp):
        junction = self.junctions.get(int(junction_id))
        room = None if junction is None else junction.get("rooms", {}).get(int(branch_id))
        if room is None or room["state"] != "observing":
            return False
        values = []
        for detection in detections or ():
            if isinstance(detection, dict):
                label, score = detection.get("label"), detection.get("score", 0.0)
            else:
                label = getattr(detection, "label", "")
                score = getattr(detection, "score", 0.0)
            label = str(label or "").strip()
            try:
                score = float(score)
            except (TypeError, ValueError):
                continue
            if label and math.isfinite(score):
                values.append({"label": label, "score": score})
        signature = tuple(sorted(set(item["label"].lower() for item in values)))
        if not signature:
            return False
        room["frames"].append({
            "stamp": float(stamp), "signature": signature, "detections": values,
        })
        room["frames"] = room["frames"][-90:]
        return True

    def _stable_run(self, frames):
        runs = []
        start = 0
        while start < len(frames):
            end = start + 1
            while end < len(frames) and frames[end]["signature"] == frames[start]["signature"]:
                end += 1
            runs.append((start, end))
            start = end
        if not runs:
            return []
        longest = max(end - start for start, end in runs)
        candidates = [(start, end) for start, end in runs if end - start == longest]
        center = (len(frames) - 1) / 2.0
        start, end = min(candidates, key=lambda run: abs((run[0] + run[1] - 1) / 2.0 - center))
        window_size = max(1, int(math.ceil((end - start) * self.stable_window_ratio)))
        middle = (start + end - 1) / 2.0
        left = max(start, min(end - window_size, int(round(middle - (window_size - 1) / 2.0))))
        return frames[left:left + window_size]

    def finish_room(self, junction_id, branch_id, *, valid, place_id=None, work_item_ids=None):
        junction = self.junctions.get(int(junction_id))
        room = None if junction is None else junction.get("rooms", {}).get(int(branch_id))
        if room is None:
            return None
        room["invalidated"] = not bool(valid)
        room["place_id"] = None if place_id is None else int(place_id)
        room["work_item_ids"] = sorted(set(int(value) for value in (work_item_ids or ())))
        if not valid:
            room["state"] = "invalidated"
            room["exploration_suppressed"] = True
            room["semantic_score"] = self.negative_score
            room["semantic_score_source"] = "invalid"
            room["labels"] = []
            room["stable_frame_count"] = 0
            return room
        stable = self._stable_run(room["frames"])
        detections = [item for frame in stable for item in frame["detections"]]
        room["stable_frame_count"] = len(stable)
        room["labels"] = sorted(set(item["label"] for item in detections))
        detector_score = (
            self.negative_score if not detections
            else sum(item["score"] for item in detections) / len(detections)
        )
        room["detector_confidence_score"] = detector_score
        task_score = None
        if detections and callable(self.semantic_scorer):
            try:
                task_score = self.semantic_scorer(room["labels"])
            except (TypeError, ValueError):
                task_score = None
        if task_score is None:
            room["semantic_score"] = detector_score
            room["semantic_score_source"] = (
                "unavailable" if not detections else "detector_confidence"
            )
        else:
            room["semantic_score"] = float(task_score)
            room["semantic_score_source"] = "task_environment"
        # A completed physical certification is valid even when WeDetect
        # supplied no stable labels. Unknown semantics ranks last; it is not
        # evidence that this child subtree has been explored or rejected.
        room["state"] = "observed"
        room["semantic_evidence"] = "stable_detections" if detections else "unavailable"
        room["exploration_suppressed"] = False
        return room

    def finish_junction(self, junction_id):
        junction = self.junctions.get(int(junction_id))
        if junction is None:
            return []
        rooms = list(junction["rooms"].values())
        rooms.sort(key=lambda room: (
            room.get("semantic_score") if room.get("semantic_score") is not None else self.negative_score,
            -room["branch_id"],
        ), reverse=True)
        for rank, room in enumerate(rooms, start=1):
            room["priority"] = rank
            score = room.get("semantic_score")
            if score is None:
                # A GP proposal can be superseded before it receives an
                # authentication transaction. Preserve the junction lineage,
                # but never treat an unobserved child as a positive room.
                score = self.negative_score
                room["semantic_score"] = score
            if score < 0 and room.get("state") != "observed":
                room["state"] = "pruned"
                room["exploration_suppressed"] = True
        junction["phase"] = "prioritized"
        return rooms

    def snapshot(self):
        return {
            int(junction_id): {
                "junction_id": junction["junction_id"],
                "phase": junction["phase"],
                "rooms": [
                    {key: value for key, value in room.items() if key != "frames"}
                    for key, room in sorted(junction["rooms"].items())
                ],
            }
            for junction_id, junction in sorted(self.junctions.items())
        }
