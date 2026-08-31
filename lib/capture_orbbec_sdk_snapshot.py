#!/usr/bin/env python3
"""Capture one depth-to-color aligned Orbbec RGB-D pair via OrbbecSDK."""

import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from orbbec_sdk_camera import OrbbecSDKCamera


def make_depth_preview(depth_mm: np.ndarray) -> np.ndarray:
    valid = depth_mm > 0
    preview = np.zeros((*depth_mm.shape, 3), dtype=np.uint8)
    if not np.any(valid):
        return preview
    low, high = np.percentile(depth_mm[valid], [2.0, 98.0])
    if high <= low:
        high = low + 1.0
    normalized = np.clip((depth_mm.astype(np.float32) - low) / (high - low), 0, 1)
    gray = (normalized * 255).astype(np.uint8)
    preview = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    preview[~valid] = 0
    return preview


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path.home() / "piper_eye_to_hand" / "board_config_sdk.yaml",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path.home() / "piper_eye_to_hand" / "captures_sdk",
    )
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--frame-timeout-ms", type=int, default=1500)
    args = parser.parse_args()
    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    with OrbbecSDKCamera(
        serial_number=str(config["camera_serial"]),
        color_width=int(config.get("color_width", 1280)),
        color_height=int(config.get("color_height", 720)),
        fps=int(config.get("camera_fps", 30)),
        warmup_frames=max(0, int(args.warmup_frames)),
    ) as camera:
        frame = camera.wait_for_rgbd(timeout_ms=max(1, int(args.frame_timeout_ms)))
        device = {
            "name": camera.device_name,
            "serial_number": camera.serial_number,
            "connection_type": camera.connection_type,
            "color_profile": camera.color_profile,
            "depth_profile_before_alignment": camera.depth_profile,
        }

    out = args.output_root.expanduser() / datetime.now().strftime("snapshot_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(out / "color.png"), frame.color_bgr)
    cv2.imwrite(str(out / "depth_mm.png"), frame.depth_mm)
    cv2.imwrite(str(out / "depth_preview.png"), make_depth_preview(frame.depth_mm))
    delta_ms = abs(frame.color_timestamp_us - frame.depth_timestamp_us) * 0.001
    metadata = {
        "capture_backend": "pyorbbecsdk2",
        "device": device,
        "color_frame_id": str(config["camera_frame"]),
        "depth_frame_id": str(config["camera_frame"]),
        "color_encoding": "bgr8",
        "depth_encoding": "16UC1",
        "width": int(frame.intrinsics.width),
        "height": int(frame.intrinsics.height),
        "distortion_model": frame.intrinsics.distortion_model,
        "D": frame.intrinsics.D,
        "K": frame.intrinsics.K,
        "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "P": [
            frame.intrinsics.K[0],
            0.0,
            frame.intrinsics.K[2],
            0.0,
            0.0,
            frame.intrinsics.K[4],
            frame.intrinsics.K[5],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
        "color_timestamp_us": frame.color_timestamp_us,
        "depth_timestamp_us": frame.depth_timestamp_us,
        "color_depth_stamp_delta_ms": delta_ms,
        "depth_storage": (
            "uint16 PNG in millimetres after SDK depth scale and software D2C alignment"
        ),
    }
    with (out / "camera_info.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False, allow_unicode=True)

    valid_ratio = float(np.count_nonzero(frame.depth_mm)) / float(frame.depth_mm.size)
    print(f"Saved snapshot: {out}")
    print(f"Camera: {camera.device_name}, serial={camera.serial_number}")
    print(f"Aligned image size: {frame.color_bgr.shape[1]}x{frame.color_bgr.shape[0]}")
    print(f"Color/depth timestamp delta: {delta_ms:.3f} ms")
    print(f"Valid depth pixels: {valid_ratio * 100.0:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
