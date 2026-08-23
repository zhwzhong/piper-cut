#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_latest_capture(images_dir):
    metas = sorted(Path(images_dir).glob("meta_*.yaml"))
    if not metas:
        raise FileNotFoundError(f"No meta_*.yaml found in {images_dir}")
    meta_path = metas[-1]
    with meta_path.open("r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    rgb_path = meta_path.parent / meta["color"]["file"]
    depth_path = meta_path.parent / meta["depth"]["file"]
    return rgb_path, depth_path, meta_path


def read_meta(meta_path):
    if meta_path is None:
        return None
    with Path(meta_path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resize_depth_to_rgb(depth, rgb_shape):
    h, w = rgb_shape[:2]
    if depth.shape[:2] == (h, w):
        return depth
    return cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)


def roi_to_slice(roi, width, height):
    if roi is None:
        return slice(0, height), slice(0, width)
    x, y, w, h = roi
    x0 = max(0, int(x))
    y0 = max(0, int(y))
    x1 = min(width, int(x + w))
    y1 = min(height, int(y + h))
    return slice(y0, y1), slice(x0, x1)


def estimate_top_mask(depth_rgb, roi=None, depth_band=60.0):
    h, w = depth_rgb.shape[:2]
    ys, xs = roi_to_slice(roi, w, h)
    valid_roi = depth_rgb[ys, xs]
    valid = valid_roi[valid_roi > 0]
    if valid.size < 1000:
        raise RuntimeError("Not enough valid depth pixels. Check camera depth topic or ROI.")

    low, high = np.percentile(valid, [5, 95])
    valid = valid[(valid >= low) & (valid <= high)]
    hist, edges = np.histogram(valid, bins=80)
    peak = int(np.argmax(hist))
    top_depth = float((edges[peak] + edges[peak + 1]) * 0.5)

    mask = np.zeros_like(depth_rgb, dtype=np.uint8)
    in_band = (depth_rgb > 0) & (np.abs(depth_rgb.astype(np.float32) - top_depth) <= depth_band)
    mask[in_band] = 255
    if roi is not None:
        roi_mask = np.zeros_like(mask)
        roi_mask[ys, xs] = 255
        mask = cv2.bitwise_and(mask, roi_mask)

    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        raise RuntimeError("Failed to segment a depth top-plane component.")
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = np.where(labels == largest, 255, 0).astype(np.uint8)
    return mask, top_depth


def estimate_raised_object_mask(depth_rgb, roi=None, min_height_mm=25.0, table_percentile=80.0):
    if depth_rgb is None:
        return None, {"enabled": False, "reason": "no_depth"}
    if depth_rgb.ndim == 3:
        depth_rgb = cv2.cvtColor(depth_rgb, cv2.COLOR_BGR2GRAY)

    h, w = depth_rgb.shape[:2]
    ys, xs = roi_to_slice(roi, w, h)
    valid_roi = depth_rgb[ys, xs]
    valid = valid_roi[valid_roi > 0]
    if valid.size < 1000:
        return None, {"enabled": False, "reason": "not_enough_valid_depth"}

    table_depth = float(np.percentile(valid, float(table_percentile)))
    near_limit = float(np.percentile(valid, 1))
    raised = (depth_rgb > max(1.0, near_limit - 50.0)) & (
        depth_rgb.astype(np.float32) < table_depth - float(min_height_mm)
    )

    mask = np.zeros_like(depth_rgb, dtype=np.uint8)
    mask[raised] = 255
    if roi is not None:
        roi_mask = np.zeros_like(mask)
        roi_mask[ys, xs] = 255
        mask = cv2.bitwise_and(mask, roi_mask)

    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask, {
        "enabled": True,
        "table_depth_raw": table_depth,
        "min_height_mm": float(min_height_mm),
        "table_percentile": float(table_percentile),
        "area_px": int((mask > 0).sum()),
    }


def estimate_cardboard_mask(
    rgb,
    depth_rgb=None,
    roi=None,
    use_depth_object=False,
    depth_min_height_mm=25.0,
):
    h, w = rgb.shape[:2]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Brown cardboard is usually low-to-medium value with noticeable saturation.
    # This deliberately excludes the white board/table and most blue/green tools.
    lower1 = np.array([5, 25, 35], dtype=np.uint8)
    upper1 = np.array([38, 210, 230], dtype=np.uint8)
    lower2 = np.array([0, 30, 35], dtype=np.uint8)
    upper2 = np.array([8, 220, 210], dtype=np.uint8)
    color_mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))

    if depth_rgb is not None and depth_rgb.ndim == 3:
        depth_rgb = cv2.cvtColor(depth_rgb, cv2.COLOR_BGR2GRAY)
    if depth_rgb is not None:
        color_mask[depth_rgb <= 0] = 0

    if roi is not None:
        ys, xs = roi_to_slice(roi, w, h)
        roi_mask = np.zeros_like(color_mask)
        roi_mask[ys, xs] = 255
        color_mask = cv2.bitwise_and(color_mask, roi_mask)

    kernel = np.ones((5, 5), np.uint8)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    raised_mask = None
    depth_info = {"enabled": False}
    candidate_mask = color_mask
    if use_depth_object:
        raised_mask, depth_info = estimate_raised_object_mask(
            depth_rgb,
            roi=roi,
            min_height_mm=depth_min_height_mm,
        )
        if raised_mask is not None and int((raised_mask > 0).sum()) > 300:
            raised_dilated = cv2.dilate(raised_mask, np.ones((9, 9), np.uint8), iterations=1)
            rgbd_mask = cv2.bitwise_and(color_mask, raised_dilated)
            if int((rgbd_mask > 0).sum()) > 600:
                candidate_mask = rgbd_mask

    num, labels, stats, centroids = cv2.connectedComponentsWithStats(candidate_mask, connectivity=8)
    if num <= 1:
        raise RuntimeError("Failed to segment cardboard-colored components.")

    # The target is the middle cardboard box, not every cardboard object.
    # Bias toward the image center and reject border-touching table/background regions.
    image_center = np.array([w * 0.5, h * 0.5], dtype=np.float32)
    best_label = None
    best_score = -1.0
    candidates = []
    for label in range(1, num):
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        bw = int(stats[label, cv2.CC_STAT_WIDTH])
        bh = int(stats[label, cv2.CC_STAT_HEIGHT])
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < 1200 or bw < 35 or bh < 25:
            continue

        touches_edge = x <= 3 or y <= 3 or (x + bw) >= w - 4 or (y + bh) >= h - 4
        if touches_edge:
            continue
        if area > 0.20 * w * h:
            continue
        aspect = bw / max(float(bh), 1.0)
        if aspect < 0.45 or aspect > 6.0:
            continue

        centroid = np.array(centroids[label], dtype=np.float32)
        # The first demo assumes the target carton is the central object. This
        # gate prevents beige side panels, walls, cables, or off-board cartons
        # from being accepted when the center object is not a cardboard box.
        cx_norm = float(centroid[0] / max(float(w), 1.0))
        cy_norm = float(centroid[1] / max(float(h), 1.0))
        if not (0.25 <= cx_norm <= 0.72 and 0.25 <= cy_norm <= 0.78):
            continue

        comp_mask = np.where(labels == label, 255, 0).astype(np.uint8)
        pts = cv2.findNonZero(comp_mask)
        if pts is None or len(pts) < 80:
            continue
        rect = cv2.minAreaRect(pts)
        side_a, side_b = [float(v) for v in rect[1]]
        long_side = max(side_a, side_b)
        short_side = min(side_a, side_b)
        rect_fill = float(area / max(long_side * short_side, 1.0))
        if long_side < 65.0 or short_side < 32.0 or rect_fill < 0.28:
            continue

        dist = float(np.linalg.norm((centroid - image_center) / np.array([w, h], dtype=np.float32)))
        center_score = 1.0 / (1.0 + 4.0 * dist)
        size_score = min(area / 12000.0, 1.0)
        depth_overlap = 0.0
        if raised_mask is not None:
            comp = labels == label
            depth_overlap = float((raised_mask[comp] > 0).sum() / max(int(comp.sum()), 1))
        shape_score = min(rect_fill, 1.0)
        score = 1000.0 * center_score + 200.0 * size_score + 260.0 * depth_overlap + 120.0 * shape_score
        candidates.append(
            (
                score,
                label,
                area,
                (x, y, bw, bh),
                touches_edge,
                depth_overlap,
                (long_side, short_side, rect_fill),
            )
        )
        if score > best_score:
            best_score = score
            best_label = label

    if best_label is None:
        raise RuntimeError("No valid cardboard component found. Try --roi x,y,w,h.")

    selected = np.where(labels == best_label, 255, 0).astype(np.uint8)
    selected = cv2.morphologyEx(selected, cv2.MORPH_CLOSE, kernel, iterations=2)
    return selected, {
        "selected_label": int(best_label),
        "selected_score": float(best_score),
        "num_candidates": len(candidates),
        "mask_source": "rgbd" if candidate_mask is not color_mask else "color",
        "depth_object": depth_info,
    }


def rect_axes_from_mask(mask):
    pts = cv2.findNonZero(mask)
    if pts is None or len(pts) < 100:
        raise RuntimeError("Top mask is too small for box orientation.")
    rect = cv2.minAreaRect(pts)
    box = cv2.boxPoints(rect).astype(np.float32)
    center = np.array(rect[0], dtype=np.float32)

    edges = [box[(i + 1) % 4] - box[i] for i in range(4)]
    lengths = [float(np.linalg.norm(e)) for e in edges]
    long_idx = int(np.argmax(lengths))
    long_axis = edges[long_idx] / (lengths[long_idx] + 1e-6)
    short_axis = np.array([-long_axis[1], long_axis[0]], dtype=np.float32)

    pixels = np.column_stack(np.where(mask > 0)).astype(np.float32)
    xy = np.column_stack([pixels[:, 1], pixels[:, 0]])
    rel = xy - center[None, :]
    u = rel @ long_axis
    v = rel @ short_axis
    box_rel = box - center[None, :]
    box_u = box_rel @ long_axis
    box_v = box_rel @ short_axis
    return {
        "rect": rect,
        "box": box,
        "center": center,
        "long_axis": long_axis,
        "short_axis": short_axis,
        "u_min": float(np.percentile(u, 2)),
        "u_max": float(np.percentile(u, 98)),
        "v_min": float(np.percentile(v, 2)),
        "v_max": float(np.percentile(v, 98)),
        "u_box_min": float(np.min(box_u)),
        "u_box_max": float(np.max(box_u)),
        "v_box_min": float(np.min(box_v)),
        "v_box_max": float(np.max(box_v)),
        "orientation_info": {
            "source": "min_area_rect",
            "line_count": 0,
            "line_weight_px": 0.0,
            "confidence": 0.0,
        },
    }


def canonicalize_axis_direction(axes):
    """Make START deterministic: image-left for horizontal seams, top for vertical."""
    long_axis = np.asarray(axes["long_axis"], dtype=np.float32)
    horizontal = abs(float(long_axis[0])) >= abs(float(long_axis[1]))
    should_flip = (horizontal and long_axis[0] < 0) or (
        not horizontal and long_axis[1] < 0
    )
    if not should_flip:
        return axes

    updated = dict(axes)
    updated["long_axis"] = -np.asarray(axes["long_axis"], dtype=np.float32)
    updated["short_axis"] = -np.asarray(axes["short_axis"], dtype=np.float32)
    for prefix in ("u", "v", "u_box", "v_box"):
        old_min = float(axes[f"{prefix}_min"])
        old_max = float(axes[f"{prefix}_max"])
        updated[f"{prefix}_min"] = -old_max
        updated[f"{prefix}_max"] = -old_min
    return updated


def recompute_axes_with_long_axis(mask, axes, long_axis, orientation_info):
    long_axis = np.array(long_axis, dtype=np.float32)
    norm = float(np.linalg.norm(long_axis))
    if norm < 1e-6:
        return axes
    long_axis = long_axis / norm
    if float(long_axis @ axes["long_axis"]) < 0:
        long_axis = -long_axis
    short_axis = np.array([-long_axis[1], long_axis[0]], dtype=np.float32)

    pixels = np.column_stack(np.where(mask > 0)).astype(np.float32)
    xy = np.column_stack([pixels[:, 1], pixels[:, 0]])
    center = axes["center"]
    rel = xy - center[None, :]
    u = rel @ long_axis
    v = rel @ short_axis
    box_rel = axes["box"] - center[None, :]
    box_u = box_rel @ long_axis
    box_v = box_rel @ short_axis

    updated = dict(axes)
    updated.update(
        {
            "long_axis": long_axis,
            "short_axis": short_axis,
            "u_min": float(np.percentile(u, 2)),
            "u_max": float(np.percentile(u, 98)),
            "v_min": float(np.percentile(v, 2)),
            "v_max": float(np.percentile(v, 98)),
            "u_box_min": float(np.min(box_u)),
            "u_box_max": float(np.max(box_u)),
            "v_box_min": float(np.min(box_v)),
            "v_box_max": float(np.max(box_v)),
            "orientation_info": orientation_info,
        }
    )
    return updated


def refine_axes_by_edge_lines(
    rgb,
    mask,
    axes,
    angle_tolerance_deg=35.0,
    min_line_length_px=24.0,
    max_line_gap_px=10.0,
):
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 140)
    roi_mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
    edges = cv2.bitwise_and(edges, roi_mask)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=20,
        minLineLength=max(8, int(round(float(min_line_length_px)))),
        maxLineGap=max(2, int(round(float(max_line_gap_px)))),
    )
    if lines is None:
        return axes

    rect_axis = axes["long_axis"]
    selected = []
    tolerance_cos = math.cos(math.radians(float(angle_tolerance_deg)))
    for line in lines[:, 0, :]:
        x1, y1, x2, y2 = [float(v) for v in line]
        vec = np.array([x2 - x1, y2 - y1], dtype=np.float32)
        length = float(np.linalg.norm(vec))
        if length < float(min_line_length_px):
            continue
        unit = vec / (length + 1e-6)
        dot = float(unit @ rect_axis)
        if abs(dot) < tolerance_cos:
            continue
        if dot < 0:
            unit = -unit
        selected.append((unit, length))

    total_weight = float(sum(length for _, length in selected))
    if len(selected) < 2 or total_weight < 45.0:
        return axes

    avg = np.zeros(2, dtype=np.float32)
    for unit, length in selected:
        avg += unit * float(length)
    norm = float(np.linalg.norm(avg))
    if norm < 1e-6:
        return axes

    confidence = float(np.clip(total_weight / 220.0, 0.0, 1.0))
    return recompute_axes_with_long_axis(
        mask,
        axes,
        avg / norm,
        {
            "source": "hough_edges",
            "line_count": int(len(selected)),
            "line_weight_px": total_weight,
            "confidence": confidence,
        },
    )


def find_dark_seam_offset(gray, mask, axes, search_ratio=0.35, bins=121):
    ys, xs = np.where(mask > 0)
    xy = np.column_stack([xs, ys]).astype(np.float32)
    rel = xy - axes["center"][None, :]
    u = rel @ axes["long_axis"]
    v = rel @ axes["short_axis"]

    u_len = axes["u_max"] - axes["u_min"]
    v_len = axes["v_max"] - axes["v_min"]
    if u_len <= 20 or v_len <= 20:
        raise RuntimeError("Box top component is too small or degenerate.")

    central_u = (u > axes["u_min"] + 0.12 * u_len) & (u < axes["u_max"] - 0.12 * u_len)
    central_v = np.abs(v) < search_ratio * v_len
    keep = central_u & central_v
    if int(keep.sum()) < 200:
        keep = central_u

    kept_xy = xy[keep].astype(np.int32)
    kept_v = v[keep]
    intensities = gray[kept_xy[:, 1], kept_xy[:, 0]].astype(np.float32)

    hist_edges = np.linspace(-search_ratio * v_len, search_ratio * v_len, bins + 1)
    scores = np.full(bins, np.inf, dtype=np.float32)
    counts = np.zeros(bins, dtype=np.int32)
    for i in range(bins):
        m = (kept_v >= hist_edges[i]) & (kept_v < hist_edges[i + 1])
        counts[i] = int(m.sum())
        if counts[i] > 30:
            scores[i] = float(np.percentile(intensities[m], 35))

    best = int(np.argmin(scores))
    if not np.isfinite(scores[best]):
        return 0.0, 0.0
    offset = float((hist_edges[best] + hist_edges[best + 1]) * 0.5)

    center_score = float(np.percentile(intensities, 35))
    confidence = float(np.clip((center_score - scores[best]) / 80.0, 0.0, 1.0))
    return offset, confidence


def find_gradient_refined_offset(gray, mask, axes, search_px=16.0, step_px=1.0, strip_px=2.0):
    sample_mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    ys, xs = np.where(sample_mask > 0)
    if len(xs) < 200:
        return 0.0, 0.0

    xy = np.column_stack([xs, ys]).astype(np.float32)
    rel = xy - axes["center"][None, :]
    u = rel @ axes["long_axis"]
    v = rel @ axes["short_axis"]

    u_len = axes["u_max"] - axes["u_min"]
    v_len = axes["v_max"] - axes["v_min"]
    if u_len <= 20 or v_len <= 20:
        return 0.0, 0.0

    # Do not use the box border as the seam. Only evaluate the middle part.
    central_u = (u > axes["u_min"] + 0.18 * u_len) & (u < axes["u_max"] - 0.18 * u_len)
    max_shift = min(float(search_px), 0.35 * v_len)
    near_center = np.abs(v) <= max_shift + strip_px + 2.0
    keep = central_u & near_center
    if int(keep.sum()) < 120:
        return 0.0, 0.0

    kept_xy = xy[keep].astype(np.int32)
    kept_v = v[keep]
    intensities = gray[kept_xy[:, 1], kept_xy[:, 0]].astype(np.float32)

    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    # Seam is parallel to long_axis; useful edge evidence is along the normal.
    normal_grad_img = np.abs(
        grad_x * float(axes["short_axis"][0]) + grad_y * float(axes["short_axis"][1])
    )
    normal_grad = normal_grad_img[kept_xy[:, 1], kept_xy[:, 0]].astype(np.float32)

    baseline_dark = float(np.percentile(intensities, 55))
    candidates = np.arange(-max_shift, max_shift + 0.5 * step_px, step_px, dtype=np.float32)
    best_offset = 0.0
    best_score = -np.inf
    second_score = -np.inf

    for off in candidates:
        m = np.abs(kept_v - float(off)) <= strip_px
        if int(m.sum()) < max(25, int(0.20 * u_len)):
            continue
        line_intensity = float(np.percentile(intensities[m], 35))
        line_grad = float(np.percentile(normal_grad[m], 80))
        dark_score = max(0.0, baseline_dark - line_intensity)
        # Penalize large shifts slightly; this is a calibration around the geometric center.
        center_penalty = 0.18 * abs(float(off))
        score = 0.75 * line_grad + 0.45 * dark_score - center_penalty
        if score > best_score:
            second_score = best_score
            best_score = score
            best_offset = float(off)
        elif score > second_score:
            second_score = score

    if not np.isfinite(best_score):
        return 0.0, 0.0

    confidence = 0.0
    if np.isfinite(second_score):
        confidence = float(np.clip((best_score - second_score) / 25.0, 0.0, 1.0))
    return best_offset, confidence


def find_line_model_offset(gray, mask, axes, search_px=18.0, step_px=1.0, strip_px=2.0):
    sample_mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
    ys, xs = np.where(sample_mask > 0)
    if len(xs) < 200:
        return 0.0, 0.0, {"source": "fallback", "reason": "not_enough_pixels"}

    xy = np.column_stack([xs, ys]).astype(np.float32)
    rel = xy - axes["center"][None, :]
    u = rel @ axes["long_axis"]
    v = rel @ axes["short_axis"]

    u_len = axes["u_max"] - axes["u_min"]
    v_len = axes["v_max"] - axes["v_min"]
    if u_len <= 20 or v_len <= 20:
        return 0.0, 0.0, {"source": "fallback", "reason": "degenerate_box"}

    u0 = axes["u_min"] + 0.15 * u_len
    u1 = axes["u_max"] - 0.15 * u_len
    max_shift = min(float(search_px), 0.35 * v_len)
    keep = (u >= u0) & (u <= u1) & (np.abs(v) <= max_shift + strip_px + 2.0)
    if int(keep.sum()) < 120:
        return 0.0, 0.0, {"source": "fallback", "reason": "not_enough_center_pixels"}

    kept_xy = xy[keep].astype(np.int32)
    kept_u = u[keep]
    kept_v = v[keep]
    intensities = gray[kept_xy[:, 1], kept_xy[:, 0]].astype(np.float32)

    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    normal_grad_img = np.abs(
        grad_x * float(axes["short_axis"][0]) + grad_y * float(axes["short_axis"][1])
    )
    normal_grad = normal_grad_img[kept_xy[:, 1], kept_xy[:, 0]].astype(np.float32)

    baseline_dark = float(np.percentile(intensities, 55))
    grad_baseline = float(np.percentile(normal_grad, 55))
    candidates = np.arange(-max_shift, max_shift + 0.5 * step_px, step_px, dtype=np.float32)
    u_edges = np.linspace(u0, u1, 22)
    best = {"offset": 0.0, "score": -np.inf, "continuity": 0.0}
    second_score = -np.inf

    for off in candidates:
        m = np.abs(kept_v - float(off)) <= float(strip_px)
        if int(m.sum()) < max(25, int(0.20 * u_len)):
            continue

        line_intensity = float(np.percentile(intensities[m], 35))
        line_grad = float(np.percentile(normal_grad[m], 80))
        dark_score = max(0.0, baseline_dark - line_intensity)
        grad_score = max(0.0, line_grad - 0.35 * grad_baseline)

        good_bins = 0
        filled_bins = 0
        for lo, hi in zip(u_edges[:-1], u_edges[1:]):
            bm = m & (kept_u >= lo) & (kept_u < hi)
            if int(bm.sum()) < 3:
                continue
            filled_bins += 1
            bin_dark = max(0.0, baseline_dark - float(np.percentile(intensities[bm], 35)))
            bin_grad = max(0.0, float(np.percentile(normal_grad[bm], 75)) - 0.35 * grad_baseline)
            if 0.45 * bin_grad + 0.55 * bin_dark > 4.0:
                good_bins += 1
        continuity = good_bins / max(filled_bins, 1)

        center_penalty = 0.16 * abs(float(off))
        score = 0.65 * grad_score + 0.55 * dark_score + 18.0 * continuity - center_penalty
        if score > best["score"]:
            second_score = best["score"]
            best = {
                "offset": float(off),
                "score": float(score),
                "continuity": float(continuity),
                "line_grad_score": float(grad_score),
                "dark_score": float(dark_score),
                "filled_bins": int(filled_bins),
                "good_bins": int(good_bins),
            }
        elif score > second_score:
            second_score = float(score)

    if not np.isfinite(best["score"]):
        return 0.0, 0.0, {"source": "fallback", "reason": "no_valid_candidate"}

    confidence = 0.0
    if np.isfinite(second_score):
        confidence = float(np.clip((best["score"] - second_score) / 18.0, 0.0, 1.0))
    confidence = float(max(confidence, min(1.0, best["continuity"] * 0.65)))
    best["source"] = "line_model"
    best["second_score"] = float(second_score) if np.isfinite(second_score) else None
    return float(best["offset"]), confidence, best


def refine_line_endpoints_by_gradient(
    gray,
    axes,
    offset,
    fallback_start,
    fallback_end,
    search_out_px=28.0,
    search_in_px=34.0,
    strip_px=2.0,
    min_confidence=0.04,
):
    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    # Left/right seam endpoints are box-side edges, so the strongest useful
    # evidence is the image gradient along the seam direction.
    edge_grad = np.abs(
        grad_x * float(axes["long_axis"][0]) + grad_y * float(axes["long_axis"][1])
    )

    center = axes["center"]
    long_axis = axes["long_axis"]
    short_axis = axes["short_axis"]
    p0 = center + float(offset) * short_axis
    h, w = gray.shape[:2]
    normal_offsets = np.arange(
        -float(strip_px),
        float(strip_px) + 0.5,
        1.0,
        dtype=np.float32,
    )

    def score_at_u(u):
        vals = []
        for dv in normal_offsets:
            p = p0 + float(u) * long_axis + float(dv) * short_axis
            x = int(round(float(p[0])))
            y = int(round(float(p[1])))
            if 0 <= x < w and 0 <= y < h:
                vals.append(float(edge_grad[y, x]))
        if not vals:
            return 0.0
        return float(np.percentile(vals, 80))

    def best_edge_u(u_values, nominal_u):
        best_u = None
        best_score = -np.inf
        second_score = -np.inf
        for u in u_values:
            score = score_at_u(u)
            # Prefer an actual edge close to the projected box side, but allow
            # a small shift when RGB gradients show the visible boundary better.
            score -= 0.10 * abs(float(u) - float(nominal_u))
            if score > best_score:
                second_score = best_score
                best_score = score
                best_u = float(u)
            elif score > second_score:
                second_score = score
        if best_u is None or not np.isfinite(best_score):
            return None, 0.0, 0.0
        confidence = 0.0
        if np.isfinite(second_score):
            confidence = float(np.clip((best_score - second_score) / 20.0, 0.0, 1.0))
        return best_u, float(best_score), confidence

    u_min = float(axes.get("u_box_min", axes["u_min"]))
    u_max = float(axes.get("u_box_max", axes["u_max"]))
    left_us = np.arange(u_min - float(search_out_px), u_min + float(search_in_px) + 0.5, 1.0)
    right_us = np.arange(u_max - float(search_in_px), u_max + float(search_out_px) + 0.5, 1.0)
    start_u, start_score, start_conf = best_edge_u(left_us, u_min)
    end_u, end_score, end_conf = best_edge_u(right_us, u_max)

    start_used_fallback = bool(start_u is None or start_conf < float(min_confidence))
    end_used_fallback = bool(end_u is None or end_conf < float(min_confidence))
    if start_used_fallback:
        p_start = np.array(fallback_start, dtype=np.float32)
    else:
        p_start = p0 + start_u * long_axis
    if end_used_fallback:
        p_end = np.array(fallback_end, dtype=np.float32)
    else:
        p_end = p0 + end_u * long_axis

    return p_start, p_end, {
        "start_u": start_u,
        "end_u": end_u,
        "start_score": start_score,
        "end_score": end_score,
        "start_confidence": start_conf,
        "end_confidence": end_conf,
        "start_used_fallback": start_used_fallback,
        "end_used_fallback": end_used_fallback,
    }


def line_endpoints_from_axes(axes, offset):
    p0 = axes["center"] + offset * axes["short_axis"]
    p_start = p0 + axes["u_min"] * axes["long_axis"]
    p_end = p0 + axes["u_max"] * axes["long_axis"]
    return p_start, p_end


def center_prior_line(axes, shrink=0.08):
    u_min = axes["u_min"]
    u_max = axes["u_max"]
    span = u_max - u_min
    u0 = u_min + shrink * span
    u1 = u_max - shrink * span
    p_start = axes["center"] + u0 * axes["long_axis"]
    p_end = axes["center"] + u1 * axes["long_axis"]
    return p_start, p_end


def extend_line_asymmetric(p_start, p_end, extend_start_px=14.0, extend_end_px=0.0):
    p_start = np.array(p_start, dtype=np.float32)
    p_end = np.array(p_end, dtype=np.float32)
    direction = p_end - p_start
    norm = float(np.linalg.norm(direction))
    if norm < 1e-6:
        return p_start, p_end
    unit = direction / norm
    return p_start - float(extend_start_px) * unit, p_end + float(extend_end_px) * unit


def clip_line_to_box_axes(p_start, p_end, axes, margin_px=3.0, start_margin_px=None, end_margin_px=None):
    p_start = np.array(p_start, dtype=np.float32)
    p_end = np.array(p_end, dtype=np.float32)
    center = axes["center"]
    long_axis = axes["long_axis"]

    start_margin = float(margin_px if start_margin_px is None else start_margin_px)
    end_margin = float(margin_px if end_margin_px is None else end_margin_px)
    min_u = float(axes.get("u_box_min", axes["u_min"])) + start_margin
    max_u = float(axes.get("u_box_max", axes["u_max"])) - end_margin
    if max_u <= min_u:
        return p_start, p_end

    def clamp_point(p):
        rel = p - center
        u = float(rel @ long_axis)
        u_clamped = float(np.clip(u, min_u, max_u))
        return p + (u_clamped - u) * long_axis

    return clamp_point(p_start), clamp_point(p_end)


def median_depth_around(depth, xy, radius=5):
    x, y = int(round(float(xy[0]))), int(round(float(xy[1])))
    h, w = depth.shape[:2]
    x0, x1 = max(0, x - radius), min(w, x + radius + 1)
    y0, y1 = max(0, y - radius), min(h, y + radius + 1)
    patch = depth[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def pixel_to_camera_xyz(x, y, z, camera_info):
    if camera_info is None or z is None:
        return None
    k = camera_info.get("k") or camera_info.get("K")
    if not k or len(k) < 6:
        return None
    camera_matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
    if camera_matrix[0, 0] == 0 or camera_matrix[1, 1] == 0:
        return None
    distortion = camera_info.get("d") or camera_info.get("D") or []
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    # The registered Orbbec 16UC1 depth is in millimetres. Undistort the RGB
    # pixel before multiplying its normalized ray by Z; this is the same
    # deprojection convention used by the eye-to-hand depth validation.
    pixel = np.asarray([[[float(x), float(y)]]], dtype=np.float64)
    normalized = cv2.undistortPoints(
        pixel,
        camera_matrix,
        distortion if distortion.size else None,
    )[0, 0]
    z_m = float(z) / 1000.0
    return [
        float(normalized[0] * z_m),
        float(normalized[1] * z_m),
        z_m,
    ]


def draw_result(
    rgb,
    mask,
    axes,
    p_start,
    p_end,
    out_path,
    box_draw_offset=(0, 0),
    line_thickness=1,
    endpoint_radius=2,
    draw_box=False,
    draw_mask=False,
):
    overlay = rgb.copy()
    if draw_box:
        box_offset = np.array(box_draw_offset, dtype=np.float32)
        box = np.round(axes["box"] + box_offset[None, :]).astype(np.int32)
        cv2.polylines(overlay, [box], True, (0, 200, 255), 2)
    if draw_mask:
        color_mask = np.zeros_like(overlay)
        color_mask[:, :, 1] = mask
        overlay = cv2.addWeighted(overlay, 0.88, color_mask, 0.12, 0)

    a = tuple(np.round(p_start).astype(int))
    b = tuple(np.round(p_end).astype(int))
    cv2.line(overlay, a, b, (0, 0, 255), int(line_thickness))
    cv2.circle(overlay, a, int(endpoint_radius), (0, 255, 0), -1)
    cv2.circle(overlay, b, int(endpoint_radius), (255, 0, 0), -1)
    height, width = overlay.shape[:2]

    def label_origin(point, text):
        text_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0][0]
        return (
            max(0, min(width - text_width - 2, int(point[0]) + 7)),
            max(18, min(height - 3, int(point[1]) - 7)),
        )

    cv2.putText(
        overlay,
        "START",
        label_origin(a, "START"),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        overlay,
        "END",
        label_origin(b, "END"),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 0, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(str(out_path), overlay)


def parse_roi(s):
    if not s:
        return None
    vals = [int(v.strip()) for v in s.split(",")]
    if len(vals) != 4:
        raise argparse.ArgumentTypeError("ROI must be x,y,w,h")
    return vals


def parse_xy_offset(s):
    vals = [int(v.strip()) for v in s.split(",")]
    if len(vals) != 2:
        raise argparse.ArgumentTypeError("Offset must be dx,dy")
    return vals


def default_lower_center_roi(rgb_shape):
    h, w = rgb_shape[:2]
    x = int(0.34 * w)
    y = int(0.58 * h)
    rw = int(0.36 * w)
    rh = int(0.32 * h)
    return [x, y, rw, rh]


def main():
    parser = argparse.ArgumentParser(description="Detect the center seam line on a cardboard box top using RGB-D.")
    parser.add_argument("--images-dir", default="images")
    parser.add_argument("--rgb")
    parser.add_argument("--depth")
    parser.add_argument("--meta")
    parser.add_argument("--roi", type=parse_roi, default=None, help="Optional RGB ROI: x,y,w,h")
    parser.add_argument(
        "--auto-roi",
        choices=["lower-center", "none"],
        default="none",
        help="Default prior for the target box location when --roi is not provided.",
    )
    parser.add_argument("--mask-mode", choices=["rgbd", "cardboard", "depth"], default="rgbd")
    parser.add_argument("--depth-min-height-mm", type=float, default=25.0, help="Minimum height over the table plane used by RGB-D cardboard masking.")
    parser.add_argument("--orientation-mode", choices=["edge", "rect"], default="edge", help="edge refines box direction with Hough lines; rect uses minAreaRect only.")
    parser.add_argument(
        "--seam-mode",
        choices=["center-prior", "dark-refine", "gradient-refine", "line-model"],
        default="line-model",
        help="line-model scores full parallel-line candidates; gradient-refine uses local gradient adjustment.",
    )
    parser.add_argument("--gradient-search-px", type=float, default=16.0)
    parser.add_argument("--gradient-strip-px", type=float, default=2.0)
    parser.add_argument("--draw-box-offset", type=parse_xy_offset, default=[0, 0], help="Overlay-only yellow box shift dx,dy in pixels.")
    parser.add_argument("--line-thickness", type=int, default=1, help="Overlay line thickness in pixels.")
    parser.add_argument("--line-extend-start-px", type=float, default=26.0, help="Extend the detected seam start point outward in pixels.")
    parser.add_argument("--line-extend-end-px", type=float, default=18.0, help="Extend the detected seam end point outward in pixels.")
    parser.add_argument("--line-edge-margin-px", type=float, default=-12.0, help="Keep seam endpoints near the detected box edge; negative values allow slight visual extension.")
    parser.add_argument("--line-start-edge-margin-px", type=float, default=-24.0, help="Start-side edge margin; more negative extends the left/start endpoint.")
    parser.add_argument("--line-end-edge-margin-px", type=float, default=-12.0, help="End-side edge margin; more negative extends the right/end endpoint.")
    parser.add_argument("--endpoint-mode", choices=["gradient", "box"], default="gradient", help="gradient finds left/right endpoints from edge gradients; box uses fixed extension and clipping.")
    parser.add_argument("--endpoint-search-out-px", type=float, default=28.0, help="Endpoint gradient search outside the projected box edge.")
    parser.add_argument("--endpoint-search-in-px", type=float, default=34.0, help="Endpoint gradient search inside the projected box edge.")
    parser.add_argument("--endpoint-gradient-strip-px", type=float, default=2.0, help="Half-width normal strip used to score endpoint gradients.")
    parser.add_argument("--endpoint-min-confidence", type=float, default=0.04, help="If endpoint gradient confidence is lower, fall back to box-boundary endpoints.")
    parser.add_argument("--endpoint-radius", type=int, default=2, help="Overlay endpoint radius in pixels.")
    parser.add_argument("--draw-box", action="store_true", help="Draw the detected box rectangle on the overlay.")
    parser.add_argument("--draw-mask", action="store_true", help="Draw the cardboard segmentation mask on the overlay.")
    parser.add_argument("--depth-band", type=float, default=60.0, help="Depth band around top plane, usually millimeters")
    parser.add_argument("--out-dir", default="outputs")
    parser.add_argument("--output-stem", default=None, help="Optional suffix stem for output files, usually the capture timestamp.")
    args = parser.parse_args()

    if args.rgb and args.depth:
        rgb_path = Path(args.rgb)
        depth_path = Path(args.depth)
        meta_path = Path(args.meta) if args.meta else None
    else:
        rgb_path, depth_path, meta_path = load_latest_capture(args.images_dir)

    meta = read_meta(meta_path) if meta_path else None
    rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if rgb is None:
        raise FileNotFoundError(rgb_path)
    if depth is None:
        raise FileNotFoundError(depth_path)

    roi = args.roi
    if roi is None and args.auto_roi == "lower-center":
        roi = default_lower_center_roi(rgb.shape)

    depth_rgb = resize_depth_to_rgb(depth, rgb.shape)
    mask_info = {}
    top_depth = None
    if args.mask_mode in ("rgbd", "cardboard"):
        mask, mask_info = estimate_cardboard_mask(
            rgb,
            depth_rgb,
            roi,
            use_depth_object=args.mask_mode == "rgbd",
            depth_min_height_mm=args.depth_min_height_mm,
        )
    else:
        mask, top_depth = estimate_top_mask(depth_rgb, roi, args.depth_band)
    axes = rect_axes_from_mask(mask)
    if args.orientation_mode == "edge":
        axes = refine_axes_by_edge_lines(rgb, mask, axes)
    axes = canonicalize_axis_direction(axes)

    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    offset = 0.0
    dark_conf = 0.0
    if args.seam_mode == "dark-refine":
        offset, dark_conf = find_dark_seam_offset(gray, mask, axes)
        p_start, p_end = line_endpoints_from_axes(axes, offset)
    elif args.seam_mode == "gradient-refine":
        offset, dark_conf = find_gradient_refined_offset(
            gray,
            mask,
            axes,
            search_px=args.gradient_search_px,
            strip_px=args.gradient_strip_px,
        )
        p_start, p_end = line_endpoints_from_axes(axes, offset)
    elif args.seam_mode == "line-model":
        offset, dark_conf, line_model_info = find_line_model_offset(
            gray,
            mask,
            axes,
            search_px=args.gradient_search_px,
            strip_px=args.gradient_strip_px,
        )
        p_start, p_end = line_endpoints_from_axes(axes, offset)
    else:
        p_start, p_end = center_prior_line(axes)
    raw_p_start = np.array(p_start, dtype=np.float32)
    raw_p_end = np.array(p_end, dtype=np.float32)
    endpoint_info = {}
    if args.endpoint_mode == "gradient":
        fallback_start, fallback_end = clip_line_to_box_axes(
            p_start,
            p_end,
            axes,
            margin_px=args.line_edge_margin_px,
            start_margin_px=args.line_start_edge_margin_px,
            end_margin_px=args.line_end_edge_margin_px,
        )
        p_start, p_end, endpoint_info = refine_line_endpoints_by_gradient(
            gray,
            axes,
            offset,
            fallback_start,
            fallback_end,
            search_out_px=args.endpoint_search_out_px,
            search_in_px=args.endpoint_search_in_px,
            strip_px=args.endpoint_gradient_strip_px,
            min_confidence=args.endpoint_min_confidence,
        )
    else:
        p_start, p_end = extend_line_asymmetric(
            p_start,
            p_end,
            extend_start_px=args.line_extend_start_px,
            extend_end_px=args.line_extend_end_px,
        )
        p_start, p_end = clip_line_to_box_axes(
            p_start,
            p_end,
            axes,
            margin_px=args.line_edge_margin_px,
            start_margin_px=args.line_start_edge_margin_px,
            end_margin_px=args.line_end_edge_margin_px,
        )

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    output_suffix = ""
    if args.output_stem:
        output_suffix = f"_{Path(str(args.output_stem)).name}"
    overlay_path = out_dir / f"center_seam_overlay{output_suffix}.png"
    mask_path = out_dir / f"box_top_mask{output_suffix}.png"
    yaml_path = out_dir / f"center_seam_result{output_suffix}.yaml"
    draw_result(
        rgb,
        mask,
        axes,
        p_start,
        p_end,
        overlay_path,
        box_draw_offset=args.draw_box_offset,
        line_thickness=args.line_thickness,
        endpoint_radius=args.endpoint_radius,
        draw_box=args.draw_box,
        draw_mask=args.draw_mask,
    )
    cv2.imwrite(str(mask_path), mask)

    depth_start = median_depth_around(depth_rgb, p_start)
    depth_end = median_depth_around(depth_rgb, p_end)
    color_info = meta.get("color_camera_info") if meta else None
    start_xyz = pixel_to_camera_xyz(float(p_start[0]), float(p_start[1]), depth_start, color_info)
    end_xyz = pixel_to_camera_xyz(float(p_end[0]), float(p_end[1]), depth_end, color_info)

    result = {
        "input": {
            "rgb": str(rgb_path),
            "depth": str(depth_path),
            "meta": str(meta_path) if meta_path else None,
            "roi": roi,
        },
        "method": "cardboard_middle_box_long_edge_centerline",
        "frame_note": "camera_xyz is in the RGB camera optical frame if camera_info is available. It is not base_link.",
        "mask_mode": args.mask_mode,
        "orientation_mode": args.orientation_mode,
        "seam_mode": args.seam_mode,
        "mask_info": mask_info,
        "orientation_info": axes.get("orientation_info", {}),
        "top_depth_raw": top_depth,
        "rgb_line": {
            "raw_start_px": [float(raw_p_start[0]), float(raw_p_start[1])],
            "raw_end_px": [float(raw_p_end[0]), float(raw_p_end[1])],
            "start_px": [float(p_start[0]), float(p_start[1])],
            "end_px": [float(p_end[0]), float(p_end[1])],
        },
        "camera_xyz_m": {
            "start": start_xyz,
            "end": end_xyz,
        },
        "quality": {
            "line_refine_offset_px": float(offset),
            "line_refine_confidence": dark_conf,
            "line_model": line_model_info if args.seam_mode == "line-model" else {},
            "endpoint_refine": endpoint_info,
            "box_mask_area_px": int((mask > 0).sum()),
            "depth_start_raw": depth_start,
            "depth_end_raw": depth_end,
        },
        "outputs": {
            "overlay": str(overlay_path),
            "box_top_mask": str(mask_path),
            "draw_box_offset_px": [int(args.draw_box_offset[0]), int(args.draw_box_offset[1])],
            "line_thickness_px": int(args.line_thickness),
            "line_extend_start_px": float(args.line_extend_start_px),
            "line_extend_end_px": float(args.line_extend_end_px),
            "line_edge_margin_px": float(args.line_edge_margin_px),
            "line_start_edge_margin_px": float(args.line_start_edge_margin_px),
            "line_end_edge_margin_px": float(args.line_end_edge_margin_px),
            "endpoint_mode": args.endpoint_mode,
            "endpoint_search_out_px": float(args.endpoint_search_out_px),
            "endpoint_search_in_px": float(args.endpoint_search_in_px),
            "endpoint_gradient_strip_px": float(args.endpoint_gradient_strip_px),
            "endpoint_min_confidence": float(args.endpoint_min_confidence),
            "endpoint_radius_px": int(args.endpoint_radius),
            "draw_box": bool(args.draw_box),
            "draw_mask": bool(args.draw_mask),
        },
    }
    with yaml_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(result, f, sort_keys=False)

    print(f"Wrote {yaml_path}")
    print(f"Overlay: {overlay_path}")
    print(f"RGB seam start={result['rgb_line']['start_px']} end={result['rgb_line']['end_px']}")
    if start_xyz and end_xyz:
        print(f"Camera XYZ start={start_xyz} end={end_xyz}")


if __name__ == "__main__":
    main()
