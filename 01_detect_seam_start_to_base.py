#!/usr/bin/env python3
"""Capture RGB-D, detect the box seam, and output probe-tip base targets.

This is an orchestration entry point for the packaged SDK-only pipeline.  It
never imports piper_sdk or sends a robot command.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
LIB = ROOT / "lib"
CONFIG = ROOT / "config"


def run(command: list[str]) -> None:
    print("+", " ".join(command), flush=True)
    completed = subprocess.run(command, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed with exit code {completed.returncode}: {command[1]}"
        )


def newest_new_directory(root: Path, before: set[Path]) -> Path:
    created = sorted(
        (
            path
            for path in root.glob("snapshot_*")
            if path.is_dir() and path.resolve() not in before
        ),
        key=lambda path: path.stat().st_mtime,
    )
    if not created:
        raise RuntimeError("camera capture completed but no new snapshot directory was found")
    return created[-1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--snapshot-dir",
        type=Path,
        help="Use an existing snapshot containing color.png, depth_mm.png and camera_info.yaml",
    )
    parser.add_argument(
        "--roi",
        default="470,190,380,320",
        help="Box search ROI in RGB pixels: x,y,width,height",
    )
    parser.add_argument(
        "--mask-mode",
        choices=("rgbd", "cardboard", "depth"),
        default="rgbd",
        help="Segmentation mode passed to detect_center_seam.py",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "outputs",
    )
    parser.add_argument(
        "--depth-correction-mode",
        choices=("off", "auto", "force"),
        default="auto",
    )
    parser.add_argument(
        "--target-z-min-mm",
        type=float,
        default=99.0,
        help="Fixed Z for generated probe-tip start/end targets in base_link, in mm",
    )
    parser.add_argument(
        "--draw-box",
        action="store_true",
        help="Draw the detected box rectangle on the output overlay.",
    )
    args = parser.parse_args()

    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    if args.snapshot_dir:
        snapshot = args.snapshot_dir.expanduser().resolve()
    else:
        capture_root = output_root / "captures"
        capture_root.mkdir(parents=True, exist_ok=True)
        before = {path.resolve() for path in capture_root.glob("snapshot_*")}
        run(
            [
                sys.executable,
                str(LIB / "capture_orbbec_sdk_snapshot.py"),
                "--config",
                str(CONFIG / "runtime_config.yaml"),
                "--output-root",
                str(capture_root),
            ]
        )
        snapshot = newest_new_directory(capture_root, before)

    required = ("color.png", "depth_mm.png", "camera_info.yaml")
    missing = [name for name in required if not (snapshot / name).is_file()]
    if missing:
        raise FileNotFoundError(f"snapshot {snapshot} is missing: {missing}")

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = output_root / f"seam_run_{run_id}"
    result_dir.mkdir(parents=True, exist_ok=False)
    stem = f"{run_id}"
    detect_command = [
        sys.executable,
        str(LIB / "detect_center_seam.py"),
        "--rgb",
        str(snapshot / "color.png"),
        "--depth",
        str(snapshot / "depth_mm.png"),
        "--roi",
        args.roi,
        "--mask-mode",
        args.mask_mode,
        "--orientation-mode",
        "edge",
        "--seam-mode",
        "line-model",
        "--endpoint-mode",
        "gradient",
        "--draw-mask",
        "--line-thickness",
        "2",
        "--endpoint-radius",
        "4",
        "--out-dir",
        str(result_dir),
        "--output-stem",
        stem,
    ]
    if args.draw_box:
        detect_command.append("--draw-box")
    run(detect_command)
    seam_yaml = result_dir / f"center_seam_result_{stem}.yaml"
    overlay = result_dir / f"center_seam_overlay_{stem}.png"
    target_json = result_dir / "probe_tip_targets_base.json"
    run(
        [
            sys.executable,
            str(LIB / "seam_to_probe_tip_targets_sdk.py"),
            "--seam-yaml",
            str(seam_yaml),
            "--depth",
            str(snapshot / "depth_mm.png"),
            "--camera-info",
            str(snapshot / "camera_info.yaml"),
            "--extrinsics",
            str(CONFIG / "eye_to_hand_extrinsics.yaml"),
            "--tcp-calibration",
            str(CONFIG / "tcp_offset_m_rad.yaml"),
            "--calibration-bundle",
            str(CONFIG / "calibration_bundle.yaml"),
            "--depth-correction",
            str(CONFIG / "depth_correction.yaml"),
            "--depth-correction-mode",
            args.depth_correction_mode,
            "--target-z-min-mm",
            f"{float(args.target_z_min_mm):.6f}",
            "--output",
            str(target_json),
        ]
    )

    document = json.loads(target_json.read_text(encoding="utf-8"))
    start = document["probe_tip_contact_targets_base"]["start_m"]
    end = document["probe_tip_contact_targets_base"]["end_m"]
    print("\n=== Packaged result (no motion) ===")
    print("frame: base_link, unit: meter")
    print(f"seam start probe-tip target: {start}")
    print(f"seam end   probe-tip target: {end}")
    print(f"overlay: {overlay}")
    print(f"full result: {target_json}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
