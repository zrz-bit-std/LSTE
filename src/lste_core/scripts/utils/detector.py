#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GroundingDINO helper utilities extracted from LSTE/Data_exchange/8B-05B.py
专供 lste_det_node 使用。
"""

import cv2
import numpy as np
import os
import torch
from pathlib import Path
from torchvision.ops import box_convert
from groundingdino.util.inference import load_model, load_image, predict, annotate


def _get_workspace_root() -> Path:
    """Auto-detect workspace root from LSTE_WS env var or this file's location."""
    env = os.environ.get("LSTE_WS")
    if env:
        return Path(env)
    # This file is at WS/src/lste_core/scripts/utils/detector.py
    return Path(__file__).resolve().parents[4]


PROJECT_ROOT = _get_workspace_root() / "model"
DEFAULT_DINO_CONFIG_PATH = PROJECT_ROOT / "GroundingDINO" / "groundingdino" / "config" / "GroundingDINO_SwinB_cfg.py"
DEFAULT_DINO_WEIGHTS_PATH = PROJECT_ROOT / "GroundingDINO" / "weights" / "groundingdino_swinb_cogcoor.pth"

COLOR_KEYWORDS = {
    "red",
    "orange",
    "yellow",
    "green",
    "blue",
    "purple",
    "pink",
    "brown",
    "black",
    "white",
    "gray",
    "grey",
    "cyan",
    "magenta",
    "beige",
    "gold",
    "silver",
}

COLOR_HSV_RANGES = {
    "red": [((0, 70, 50), (8, 255, 255)), ((172, 70, 50), (180, 255, 255))],
    "orange": [((9, 120, 80), (18, 255, 255))],
    "yellow": [((19, 120, 80), (32, 255, 255))],
    "green": [((33, 60, 40), (85, 255, 255))],
    "blue": [((86, 60, 40), (125, 255, 255))],
    "purple": [((131, 50, 40), (155, 255, 255))],
    "pink": [((156, 50, 60), (169, 255, 255))],
    "brown": [((10, 60, 30), (20, 220, 160))],
    "black": [((0, 0, 0), (180, 255, 40))],
    "white": [((0, 0, 200), (180, 40, 255))],
    "gray": [((0, 0, 40), (180, 40, 200))],
    "grey": [((0, 0, 40), (180, 40, 200))],
}


def extract_color_terms(attributes):
    colors = []
    for attr in attributes or []:
        attr_lower = str(attr).lower()
        for color in COLOR_KEYWORDS:
            if color in attr_lower and color not in colors:
                colors.append(color)
    return colors


def safe_predict(model, image, caption: str, box_threshold: float, text_threshold: float, device: str = "cuda", remove_combined: bool = False):
    """
    Wrap GroundingDINO predict to treat empty candidates as合法空结果，而不是抛出 max/argmax 异常。
    """
    try:
        return predict(
            model=model,
            image=image,
            caption=caption,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=device,
            remove_combined=remove_combined,
        )
    except RuntimeError as exc:
        if "no elements" in str(exc):
            empty_boxes = torch.empty((0, 4))
            empty_scores = torch.empty((0,))
            return empty_boxes, empty_scores, []
        raise


def _cxcywh_to_xyxy_norm(box):
    cx, cy, w, h = box
    x1 = cx - 0.5 * w
    y1 = cy - 0.5 * h
    x2 = cx + 0.5 * w
    y2 = cy + 0.5 * h
    return [x1, y1, x2, y2]


def _iou_norm(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter = inter_w * inter_h
    area1 = max(0.0, box1[2] - box1[0]) * max(0.0, box1[3] - box1[1])
    area2 = max(0.0, box2[2] - box2[0]) * max(0.0, box2[3] - box2[1])
    union = area1 + area2 - inter
    if union <= 0:
        return 0.0
    return inter / union


def nms_iou(boxes, scores, phrases, threshold=0.9):
    if boxes is None:
        return boxes, scores, phrases
    boxes_list = boxes.cpu().tolist() if isinstance(boxes, torch.Tensor) else boxes
    scores_list = scores.cpu().tolist() if isinstance(scores, torch.Tensor) else [float(s) for s in scores] if scores is not None else []
    phrases_list = [str(p) for p in phrases] if phrases is not None else []
    if not boxes_list:
        return boxes, scores, phrases
    xyxy_list = [_cxcywh_to_xyxy_norm(b) for b in boxes_list]
    indices = sorted(range(len(boxes_list)), key=lambda i: scores_list[i] if i < len(scores_list) else 0.0, reverse=True)
    keep = []
    while indices:
        i = indices.pop(0)
        keep.append(i)
        remain = []
        for j in indices:
            iou = _iou_norm(xyxy_list[i], xyxy_list[j])
            if iou <= threshold:
                remain.append(j)
        indices = remain
    filtered_boxes = [boxes_list[i] for i in keep]
    filtered_scores = [scores_list[i] for i in keep] if scores_list else []
    filtered_phrases = [phrases_list[i] for i in keep] if phrases_list else []
    return torch.tensor(filtered_boxes), torch.tensor(filtered_scores), filtered_phrases


def filter_boxes_by_overlap(boxes, logits, phrases, ref_boxes, threshold=0.5):
    if boxes is None or ref_boxes is None:
        return boxes, logits, phrases
    boxes_list = boxes.cpu().tolist() if isinstance(boxes, torch.Tensor) else boxes
    logits_list = logits.cpu().tolist() if isinstance(logits, torch.Tensor) else [float(s) for s in logits] if logits is not None else []
    phrases_list = [str(p) for p in phrases] if phrases is not None else []
    ref_list = ref_boxes.cpu().tolist() if isinstance(ref_boxes, torch.Tensor) else ref_boxes
    ref_xyxy = [_cxcywh_to_xyxy_norm(b) for b in ref_list]
    keep = []
    for idx, b in enumerate(boxes_list):
        xyxy = _cxcywh_to_xyxy_norm(b)
        max_iou = max((_iou_norm(xyxy, rb) for rb in ref_xyxy), default=0.0)
        if max_iou <= threshold:
            keep.append(idx)
    filtered_boxes = [boxes_list[i] for i in keep]
    filtered_logits = [logits_list[i] for i in keep] if logits_list else []
    filtered_phrases = [phrases_list[i] for i in keep] if phrases_list else []
    return torch.tensor(filtered_boxes), torch.tensor(filtered_logits), filtered_phrases


def keep_top_confidence_detection(boxes, logits, phrases):
    if boxes is None:
        return boxes, logits, phrases
    boxes_list = boxes.cpu().tolist() if isinstance(boxes, torch.Tensor) else boxes
    logits_list = logits.cpu().tolist() if isinstance(logits, torch.Tensor) else [float(s) for s in logits] if logits is not None else []
    phrases_list = [str(p) for p in phrases] if phrases is not None else []
    if not boxes_list or len(boxes_list) <= 1:
        return boxes, logits, phrases
    if logits_list:
        max_idx = max(range(len(boxes_list)), key=lambda i: logits_list[i] if i < len(logits_list) else 0.0)
    else:
        max_idx = 0
    selected_box = boxes_list[max_idx]
    selected_logit = logits_list[max_idx] if max_idx < len(logits_list) else 0.0
    selected_phrase = phrases_list[max_idx] if max_idx < len(phrases_list) else None
    top_boxes = torch.tensor([selected_box])
    top_logits = torch.tensor([selected_logit])
    top_phrases = [] if selected_phrase is None else [selected_phrase]
    return top_boxes, top_logits, top_phrases


def _crop_box(image_source: np.ndarray, box: torch.Tensor) -> np.ndarray:
    if image_source is None or box is None:
        return None
    h, w, _ = image_source.shape
    scale = torch.tensor([w, h, w, h], dtype=torch.float32)
    scaled = (box * scale).unsqueeze(0)
    xyxy = box_convert(boxes=scaled, in_fmt="cxcywh", out_fmt="xyxy")[0]
    x1, y1, x2, y2 = xyxy.tolist()
    x1 = max(0, min(w - 1, int(x1)))
    y1 = max(0, min(h - 1, int(y1)))
    x2 = max(x1 + 1, min(w, int(x2)))
    y2 = max(y1 + 1, min(h, int(y2)))
    return image_source[y1:y2, x1:x2]


def validate_color_by_phrase(image_source, boxes, logits, phrases, ratio_threshold=0.02, blur_ksize=3, dilate_iter=1):
    """Keep detections whose named colour is visible in their BGR crop.

    ``DetectionNodeBase`` obtains ROS camera frames with ``bgr8``.  Converting
    these pixels with ``COLOR_RGB2HSV`` swaps red and blue, which made a real
    yellow cup look blue to this final validation stage and caused the model
    result to disappear before the Goal Manager could observe it.
    """
    if boxes is None or phrases is None:
        return boxes, logits, phrases
    boxes_list = boxes.cpu().tolist() if isinstance(boxes, torch.Tensor) else boxes
    logits_list = logits.cpu().tolist() if isinstance(logits, torch.Tensor) else [float(s) for s in logits] if logits is not None else []
    phrases_list = [str(p) for p in phrases]
    keep = []
    for idx, (phrase, box) in enumerate(zip(phrases_list, boxes_list)):
        phrase_low = phrase.lower()
        colors_in_phrase = [c for c in COLOR_KEYWORDS if c in phrase_low and c in COLOR_HSV_RANGES]
        if not colors_in_phrase:
            keep.append(idx)
            continue
        box_t = torch.tensor(box, dtype=torch.float32)
        crop = _crop_box(image_source, box_t)
        if crop is None or crop.size == 0:
            continue
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        if blur_ksize and blur_ksize > 1:
            hsv = cv2.GaussianBlur(hsv, (blur_ksize, blur_ksize), 0)
        matched = False
        for color in colors_in_phrase:
            for lower, upper in COLOR_HSV_RANGES[color]:
                mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
                if dilate_iter and dilate_iter > 0:
                    mask = cv2.dilate(mask, None, iterations=dilate_iter)
                ratio = float(cv2.countNonZero(mask)) / float(mask.size)
                if ratio >= ratio_threshold:
                    matched = True
                    break
            if matched:
                break
        if matched:
            keep.append(idx)
    filtered_boxes = [boxes_list[i] for i in keep]
    filtered_logits = [logits_list[i] for i in keep] if logits_list else []
    filtered_phrases = [phrases_list[i] for i in keep] if phrases_list else []
    return torch.tensor(filtered_boxes), torch.tensor(filtered_logits), filtered_phrases


def filter_boxes_by_color(image_source, boxes, logits, phrases, color_terms, ratio_threshold=0.01, min_keep=1):
    """Prefer BGR boxes with the requested colour without changing fallback.

    The caller deliberately retains the original detector result when every
    candidate fails this optional preference.  Its colour conversion must use
    the same BGR convention as ``validate_color_by_phrase`` above.
    """
    if not color_terms or boxes is None:
        return boxes, logits, phrases, False, []
    boxes_list = boxes.cpu().tolist() if isinstance(boxes, torch.Tensor) else boxes
    logits_list = logits.cpu().tolist() if isinstance(logits, torch.Tensor) else [float(s) for s in logits] if logits is not None else []
    phrases_list = [str(p) for p in phrases] if phrases is not None else []
    kept = []
    removed = []
    for idx, box in enumerate(boxes_list):
        box_t = torch.tensor(box, dtype=torch.float32)
        crop = _crop_box(image_source, box_t)
        if crop is None or crop.size == 0:
            removed.append(phrases_list[idx] if idx < len(phrases_list) else "")
            continue
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        matched = False
        for color in color_terms:
            ranges = COLOR_HSV_RANGES.get(color)
            if not ranges:
                continue
            for lower, upper in ranges:
                mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
                ratio = float(cv2.countNonZero(mask)) / float(mask.size)
                if ratio >= ratio_threshold:
                    matched = True
                    break
            if matched:
                break
        if matched:
            kept.append(idx)
        else:
            removed.append(phrases_list[idx] if idx < len(phrases_list) else "")
    if len(kept) < min_keep:
        return boxes, logits, phrases, False, []
    filtered_boxes = [boxes_list[i] for i in kept]
    filtered_logits = [logits_list[i] for i in kept] if logits_list else []
    filtered_phrases = [phrases_list[i] for i in kept] if phrases_list else []
    return torch.tensor(filtered_boxes), torch.tensor(filtered_logits), filtered_phrases, True, removed


def postprocess_target_candidates(
    image_source,
    boxes,
    logits,
    phrases,
    color_terms,
    ratio_threshold=0.02,
):
    """Apply optional visual corroboration without erasing model evidence.

    The detector model supplies semantic evidence (for example ``yellow cup``)
    and the pixel-color test is only a corroborating observation.  A small,
    blurred, occluded, or badly exposed object can legitimately fail that
    second observation even when the model candidate is useful.  The color
    filter already has the right evidence-layer contract: it narrows the
    candidates when at least one box has positive color evidence and returns
    the original candidates when none can be verified.  Keep that fallback
    instead of applying a second destructive validator to it.
    """
    if not color_terms or boxes is None or len(boxes) == 0:
        return boxes, logits, phrases
    filtered_boxes, filtered_logits, filtered_phrases, _verified, _removed = (
        filter_boxes_by_color(
            image_source,
            boxes,
            logits,
            phrases,
            color_terms,
            ratio_threshold=ratio_threshold,
            min_keep=1,
        )
    )
    return filtered_boxes, filtered_logits, filtered_phrases


def run_grounding_dino_with_caption(
    model,
    image_source,
    image,
    caption: str,
    run_label: str,
    box_threshold: float = 0.35,
    text_threshold: float = 0.25,
    output_path=None,
):
    caption = (caption or "").strip()
    if not caption:
        raise ValueError("Caption for GroundingDINO cannot be empty.")
    boxes, logits, phrases = safe_predict(
        model=model,
        image=image,
        caption=caption,
        box_threshold=box_threshold,
        text_threshold=text_threshold,
    )
    if output_path is not None:
        if len(boxes) > 0:
            annotated_frame = annotate(
                image_source=image_source,
                boxes=boxes,
                logits=logits,
                phrases=phrases,
            )
            cv2.imwrite(output_path, annotated_frame)
        else:
            cv2.imwrite(output_path, image_source)
    return boxes, logits, phrases
