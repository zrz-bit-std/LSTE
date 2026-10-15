"""Score-panel rendering for the live LSTE detection visualization."""

import cv2


class DetectionScorePanelMixin:
    """Render state and score bars without coupling to ROS subscriptions."""

    def _draw_scores(self, image, scores, state=None):
        height, width = image.shape[:2]
        margin = 10
        panel_width = min(width - 2 * margin, max(330, int(width * 0.36)))
        panel_height = min(height - 2 * margin, max(145, int(height * 0.22)))
        x0, y0 = margin, margin
        x1, y1 = x0 + panel_width, y0 + panel_height
        self._draw_score_panel_background(image, x0, y0, x1, y1)

        status, status_color, subtype = self._score_status(scores, state)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(
            image,
            f"S_total {scores.s_total:.2f}",
            (x0 + 12, y0 + 30),
            font,
            0.76,
            (230, 230, 230),
            2,
            cv2.LINE_AA,
        )
        self._draw_score_status(image, status, status_color, subtype, x0, y0, x1)

        bar_left = x0 + 14
        bar_top = y0 + (72 if subtype else 52)
        bar_width = panel_width - 28
        bar_height = max(9, int(height * 0.014))
        bar_gap = max(13, int(height * 0.018))
        metrics = (
            ("S_target", scores.s_target, 0.0, 1.0, self.palette["target"]["edge"]),
            ("S_env", scores.s_env, -1.0, 1.0, self.palette["env"]["edge"]),
            ("S_ctx", scores.s_ctx, -1.0, 1.0, self.palette["ctx"]["edge"]),
        )
        for index, metric in enumerate(metrics):
            self._draw_score_bar(
                image,
                metric,
                bar_left,
                bar_top + index * (bar_height + bar_gap),
                bar_width,
                bar_height,
            )

    @staticmethod
    def _draw_score_panel_background(image, x0, y0, x1, y1):
        overlay = image.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (12, 12, 12), -1)
        cv2.rectangle(overlay, (x0, y1 - 1), (x1, y1), (45, 45, 45), 1)
        cv2.addWeighted(overlay, 0.85, image, 0.15, 0, image)

    def _score_status(self, scores, state):
        if self.task_done:
            return "DONE", (0, 200, 0), ""
        if state is None:
            return ("LOCKED", (72, 210, 170), "") if scores.detected else ("SEARCHING", (50, 70, 220), "")
        if state.state == 1:
            raw_subtype = (state.subtype or "").strip()
            suffix = raw_subtype.split("-", 1)[-1].strip().upper() if raw_subtype else "C"
            if suffix not in ("A", "B", "C"):
                suffix = "C"
            return f"SUSPICIOUS-{suffix}", (0, 165, 255), ""
        if state.state == 2:
            return "LOCKED", (72, 210, 170), state.subtype or ""
        if state.state == 3:
            return "EXHAUSTED", (140, 140, 140), state.subtype or ""
        return "PASS", (50, 70, 220), state.subtype or ""

    @staticmethod
    def _draw_score_status(image, status, status_color, subtype, x0, y0, x1):
        font = cv2.FONT_HERSHEY_DUPLEX
        status_size, _ = cv2.getTextSize(status, font, 0.64, 2)
        status_x = max(x0 + 12, x1 - 12 - status_size[0])
        cv2.putText(image, status, (status_x, y0 + 30), font, 0.64, status_color, 2, cv2.LINE_AA)
        if subtype:
            subtype_size, _ = cv2.getTextSize(subtype, font, 0.56, 2)
            subtype_x = max(x0 + 12, x1 - 12 - subtype_size[0])
            cv2.putText(image, subtype, (subtype_x, y0 + 54), font, 0.56, (230, 230, 230), 2, cv2.LINE_AA)

    @staticmethod
    def _draw_score_bar(image, metric, left, top, width, height):
        name, value, minimum, maximum, color = metric
        bottom = top + height
        cv2.rectangle(image, (left, top), (left + width, bottom), (35, 35, 35), -1)
        ratio = max(0.0, min(1.0, (float(value) - minimum) / (maximum - minimum + 1e-9)))
        bar_length = int(width * ratio)
        if bar_length > 0:
            foreground = tuple(int(0.75 * channel + 0.25 * 255) for channel in color)
            cv2.rectangle(image, (left, top), (left + bar_length, bottom), foreground, -1)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(image, name, (left, top - 3), font, 0.58, (210, 210, 210), 2, cv2.LINE_AA)
        value_text = f"{value:+.2f}" if minimum < 0.0 else f"{value:.2f}"
        value_size, _ = cv2.getTextSize(value_text, font, 0.58, 2)
        cv2.putText(
            image,
            value_text,
            (left + width - value_size[0], top + height - 2),
            font,
            0.58,
            (240, 240, 240),
            2,
            cv2.LINE_AA,
        )
