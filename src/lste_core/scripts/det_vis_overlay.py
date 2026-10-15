"""Detection-box rendering for the live LSTE visualization image."""

import math

import cv2
import rospy


class DetectionOverlayMixin:
    """Draw semantic detection boxes; score and goal panels live elsewhere."""

    def draw_detections(self, image, detections, scores=None, state=None, image_header=None):
        if image is None:
            return image
        height, width = image.shape[:2]
        if width <= 0 or height <= 0:
            return image

        if detections is not None:
            context_terms, negative_terms = self._task_label_terms()
            self._draw_target_fill(image, detections.target_dets, width, height)
            for detection in detections.target_dets:
                self._draw_box(
                    image,
                    detection,
                    width,
                    height,
                    self._detection_color(detection, context_terms, negative_terms, target=True),
                )
            for detection in detections.env_dets:
                self._draw_box(
                    image,
                    detection,
                    width,
                    height,
                    self._detection_color(detection, context_terms, negative_terms, target=False),
                )

        if scores is not None:
            self._draw_scores(image, scores, state)
        try:
            self._draw_global_goal_indicator(image, image_header)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Draw global goal failed: %s", exc)
        return image

    def _task_label_terms(self):
        context_terms = []
        negative_terms = []
        if self.latest_task is None:
            return context_terms, negative_terms
        for key in ("ctx_left", "ctx_right"):
            value = str(getattr(self.latest_task, key, "") or "").strip().lower()
            if value and value != "none":
                context_terms.append(value)
        for term in self.latest_task.obj_negative_clues or []:
            value = str(term).strip().lower()
            if value:
                negative_terms.append(value)
        return context_terms, negative_terms

    def _draw_target_fill(self, image, detections, width, height):
        boxes = [self._norm_box_to_pixels(detection, width, height) for detection in detections]
        if not boxes:
            return
        overlay = image.copy()
        fill_color = self.palette["target"]["fill"]
        for x1, y1, x2, y2 in boxes:
            cv2.rectangle(overlay, (x1, y1), (x2, y2), fill_color, -1)
        cv2.addWeighted(overlay, 0.25, image, 0.75, 0, image)

    def _detection_color(self, detection, context_terms, negative_terms, target):
        label = (detection.label or "").lower()
        if any(term in label for term in context_terms):
            return self.palette["ctx"]["edge"]
        if not target and any(term in label for term in negative_terms):
            return self.palette["neg"]["edge"]
        return self.palette["target" if target else "env"]["edge"]

    def _draw_box(self, image, detection, width, height, color):
        x1, y1, x2, y2 = self._norm_box_to_pixels(detection, width, height)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, self.line_thickness)
        if self.draw_labels:
            label = self._build_label(detection)
            if label:
                self._draw_label(image, label, (x1, y1 - 4), color)

    @staticmethod
    def _norm_box_to_pixels(detection, width, height):
        center_x = float(detection.cx) * width
        center_y = float(detection.cy) * height
        box_width = max(1.0, float(detection.w) * width)
        box_height = max(1.0, float(detection.h) * height)
        x1 = int(max(0, min(width - 1, math.floor(center_x - 0.5 * box_width))))
        y1 = int(max(0, min(height - 1, math.floor(center_y - 0.5 * box_height))))
        x2 = int(max(0, min(width - 1, math.ceil(center_x + 0.5 * box_width))))
        y2 = int(max(0, min(height - 1, math.ceil(center_y + 0.5 * box_height))))
        if x2 <= x1:
            x2 = min(width - 1, x1 + 1)
        if y2 <= y1:
            y2 = min(height - 1, y1 + 1)
        return x1, y1, x2, y2

    @staticmethod
    def _build_label(detection):
        label = (detection.label or "").strip()
        if detection.score:
            try:
                return f"{label} {float(detection.score):.2f}" if label else f"{float(detection.score):.2f}"
            except (TypeError, ValueError):
                pass
        return label

    def _draw_label(self, image, text, origin, color):
        if not text:
            return
        font = cv2.FONT_HERSHEY_DUPLEX
        scale = self.font_scale
        thickness = max(2, self.line_thickness)
        text_size, baseline = cv2.getTextSize(text, font, scale, thickness)
        x, y = origin
        x = max(0, min(image.shape[1] - text_size[0], x))
        y = max(text_size[1], min(image.shape[0] - baseline, y))
        cv2.rectangle(
            image,
            (x, y - text_size[1] - baseline),
            (x + text_size[0], y + baseline),
            (0, 0, 0),
            thickness=-1,
        )
        cv2.putText(image, text, (x, y), font, scale, color, thickness, lineType=cv2.LINE_AA)
