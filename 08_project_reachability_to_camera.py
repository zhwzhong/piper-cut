#!/usr/bin/env python3
"""Project base_link reachability points into the camera image.

Inputs are the MoveIt reachability JSON, eye-to-hand extrinsics, camera
intrinsics, and an optional color image. The script does not require numpy,
PyYAML, OpenCV, or Pillow; it writes a CSV and an SVG overlay.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import html
import json
import math
from pathlib import Path
import re
from typing import Any


ROOT = Path(__file__).resolve().parent
DEFAULT_REACHABILITY = ROOT / "outputs" / "reachability" / "spark_tip_down_box_region.json"
DEFAULT_EXTRINSICS = ROOT / "config" / "eye_to_hand_extrinsics.yaml"
DEFAULT_CAMERA_INFO = ROOT / "calibration_handoff_20260822_194337" / "outputs" / "web_panel" / "captures" / "snapshot_20260822_095517" / "camera_info.yaml"
DEFAULT_IMAGE = ROOT / "calibration_handoff_20260822_194337" / "outputs" / "web_panel" / "captures" / "snapshot_20260822_095517" / "color.png"
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "reachability_camera"


def now_id() -> str:
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")


def parse_flat_yaml_list(text: str, key: str, expected: int) -> list[float]:
    match = re.search(rf"^{re.escape(key)}:\s*\n((?:\s*-\s*[-+0-9.eE]+\s*\n)+)", text, re.MULTILINE)
    if not match:
        raise ValueError(f"Cannot find YAML list {key}")
    values = [float(item) for item in re.findall(r"-\s*([-+0-9.eE]+)", match.group(1))]
    if len(values) != expected:
        raise ValueError(f"{key} expected {expected} values, got {len(values)}")
    return values


def parse_matrix_4x4(text: str, transform_key: str) -> list[list[float]]:
    start = text.find(f"{transform_key}:")
    if start < 0:
        raise ValueError(f"Cannot find transform {transform_key}")
    section = text[start:]
    next_key = re.search(r"\n[A-Za-z0-9_]+:\n", section[len(transform_key) + 2 :])
    if next_key:
        section = section[: len(transform_key) + 2 + next_key.start()]
    matrix_start = section.find("matrix_4x4:")
    if matrix_start < 0:
        raise ValueError(f"Cannot find {transform_key}.matrix_4x4")
    matrix_section = section[matrix_start:]
    values = []
    for line in matrix_section.splitlines()[1:]:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("-"):
            values.extend(float(item) for item in re.findall(r"[-+]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", stripped))
            if len(values) >= 16:
                break
    rows = [values[index : index + 4] for index in range(0, 16, 4)]
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError(f"{transform_key}.matrix_4x4 is incomplete")
    return rows


def load_camera_info(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    width_match = re.search(r"^width:\s*(\d+)\s*$", text, re.MULTILINE)
    height_match = re.search(r"^height:\s*(\d+)\s*$", text, re.MULTILINE)
    if not width_match or not height_match:
        raise ValueError("camera_info needs width and height")
    k = parse_flat_yaml_list(text, "K", 9)
    d = parse_flat_yaml_list(text, "D", 8)
    return {
        "width": int(width_match.group(1)),
        "height": int(height_match.group(1)),
        "fx": k[0],
        "fy": k[4],
        "cx": k[2],
        "cy": k[5],
        "distortion": d,
    }


def transform_point(matrix: list[list[float]], xyz_m: list[float]) -> list[float]:
    x, y, z = xyz_m
    return [
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
    ]


def project_pinhole(camera_xyz_m: list[float], camera: dict[str, Any]) -> tuple[float, float]:
    x, y, z = camera_xyz_m
    if z <= 1.0e-9:
        raise ValueError("point is behind camera")
    u = camera["fx"] * x / z + camera["cx"]
    v = camera["fy"] * y / z + camera["cy"]
    return u, v


def tilt_color(tilt: float | None, reachable: bool) -> str:
    if not reachable:
        return "#6b7280"
    if tilt is None:
        return "#2563eb"
    if tilt <= 0:
        return "#16a34a"
    if tilt <= 10:
        return "#84cc16"
    if tilt <= 20:
        return "#f59e0b"
    return "#ef4444"


def circle_from_points(points: list[dict[str, Any]]) -> tuple[float, float, float]:
    xs = [p["u_px"] for p in points]
    ys = [p["v_px"] for p in points]
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    radius = max(math.hypot(p["u_px"] - cx, p["v_px"] - cy) for p in points)
    return cx, cy, radius


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "base_x_mm",
                "base_y_mm",
                "base_z_mm",
                "camera_x_m",
                "camera_y_m",
                "camera_z_m",
                "u_px",
                "v_px",
                "inside_image",
                "ik_ok",
                "tip_tilt_deg",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    *row["base_xyz_mm"],
                    *row["camera_xyz_m"],
                    row["u_px"],
                    row["v_px"],
                    int(row["inside_image"]),
                    int(row["ik_ok"]),
                    row["tip_tilt_deg"],
                ]
            )


def write_svg(path: Path, rows: list[dict[str, Any]], camera: dict[str, Any], image: Path | None, title: str) -> None:
    width = camera["width"]
    height = camera["height"]
    visible_ok = [r for r in rows if r["ik_ok"] and r["inside_image"]]
    circle = circle_from_points(visible_ok) if visible_ok else None
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f"<title>{html.escape(title)}</title>",
        '<rect width="100%" height="100%" fill="#111827"/>',
    ]
    if image and image.exists():
        parts.append(f'<image href="{html.escape(image.as_posix())}" x="0" y="0" width="{width}" height="{height}" preserveAspectRatio="none" opacity="0.86"/>')
    parts.extend(
        [
            '<g fill="none" stroke="#60a5fa" stroke-width="1" opacity="0.25">',
            f'<line x1="{camera["cx"]:.2f}" y1="0" x2="{camera["cx"]:.2f}" y2="{height}"/>',
            f'<line x1="0" y1="{camera["cy"]:.2f}" x2="{width}" y2="{camera["cy"]:.2f}"/>',
            "</g>",
        ]
    )
    for row in rows:
        if not row["inside_image"]:
            continue
        radius = 2.5 if row["ik_ok"] else 1.5
        opacity = 0.72 if row["ik_ok"] else 0.25
        parts.append(
            f'<circle cx="{row["u_px"]:.2f}" cy="{row["v_px"]:.2f}" r="{radius}" '
            f'fill="{tilt_color(row["tip_tilt_deg"], row["ik_ok"])}" fill-opacity="{opacity}" stroke="none"/>'
        )
    if circle:
        cx, cy, radius = circle
        parts.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{radius + 18:.2f}" fill="none" stroke="#ef4444" stroke-width="5" stroke-opacity="0.9"/>')
        parts.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="5" fill="#ef4444"/>')
    parts.extend(
        [
            '<g font-family="Arial, sans-serif" font-size="18" font-weight="700">',
            '<rect x="16" y="16" width="490" height="116" rx="8" fill="rgba(17,24,39,0.72)"/>',
            '<text x="32" y="48" fill="#ffffff">Projected TCP reachability in camera frame</text>',
            '<text x="32" y="78" fill="#bbf7d0">green: 0 deg</text>',
            '<text x="178" y="78" fill="#bef264">lime: 10 deg</text>',
            '<text x="318" y="78" fill="#fbbf24">orange: 20 deg</text>',
            '<text x="32" y="108" fill="#fca5a5">red: 30 deg</text>',
            '<text x="178" y="108" fill="#fecaca">big red circle: approximate reachable region</text>',
            "</g>",
            "</svg>",
        ]
    )
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reachability-json", type=Path, default=DEFAULT_REACHABILITY)
    parser.add_argument("--extrinsics", type=Path, default=DEFAULT_EXTRINSICS)
    parser.add_argument("--camera-info", type=Path, default=DEFAULT_CAMERA_INFO)
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--output-stem", default="")
    parser.add_argument("--include-unreachable", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    reachability = json.loads(args.reachability_json.read_text(encoding="utf-8"))
    extrinsics_text = args.extrinsics.read_text(encoding="utf-8")
    t_camera_base = parse_matrix_4x4(extrinsics_text, "T_camera_base")
    camera = load_camera_info(args.camera_info)
    rows: list[dict[str, Any]] = []
    for point in reachability["points"]:
        ik_ok = bool(point.get("ik_ok"))
        if not ik_ok and not args.include_unreachable:
            continue
        base_xyz_mm = [float(value) for value in point["xyz_mm"]]
        base_xyz_m = [value / 1000.0 for value in base_xyz_mm]
        camera_xyz_m = transform_point(t_camera_base, base_xyz_m)
        try:
            u_px, v_px = project_pinhole(camera_xyz_m, camera)
            inside = 0.0 <= u_px < camera["width"] and 0.0 <= v_px < camera["height"]
        except ValueError:
            u_px, v_px, inside = float("nan"), float("nan"), False
        tip_down = point.get("tip_down") or {}
        rows.append(
            {
                "base_xyz_mm": base_xyz_mm,
                "camera_xyz_m": [round(value, 9) for value in camera_xyz_m],
                "u_px": round(u_px, 3) if math.isfinite(u_px) else u_px,
                "v_px": round(v_px, 3) if math.isfinite(v_px) else v_px,
                "inside_image": inside,
                "ik_ok": ik_ok,
                "tip_tilt_deg": tip_down.get("tilt_deg"),
            }
        )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.output_stem or f"reachability_camera_{now_id()}"
    csv_path = args.output_dir / f"{stem}.csv"
    json_path = args.output_dir / f"{stem}.json"
    svg_path = args.output_dir / f"{stem}.svg"
    write_csv(csv_path, rows)
    json_path.write_text(json.dumps({"camera": camera, "source": str(args.reachability_json), "points": rows}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_svg(svg_path, rows, camera, args.image, stem)
    visible_ok = sum(1 for row in rows if row["ik_ok"] and row["inside_image"])
    visible_total = sum(1 for row in rows if row["inside_image"])
    print("=== Reachability projected to camera ===")
    print(f"source: {args.reachability_json}")
    print(f"frame transform: T_camera_base from {args.extrinsics}")
    print(f"camera: {camera['width']}x{camera['height']} fx={camera['fx']:.3f} fy={camera['fy']:.3f} cx={camera['cx']:.3f} cy={camera['cy']:.3f}")
    print(f"visible projected points: {visible_total}/{len(rows)}")
    print(f"visible reachable points: {visible_ok}")
    print(f"Saved CSV:  {csv_path}")
    print(f"Saved JSON: {json_path}")
    print(f"Saved SVG:  {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
