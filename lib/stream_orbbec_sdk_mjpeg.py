#!/usr/bin/env python3
"""Stream Orbbec RGB-D color frames as MJPEG from a main-thread SDK process."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import yaml

from orbbec_sdk_camera import OrbbecSDKCamera


BOUNDARY = "piper_frame"


def binary_stdout_with_logs_on_stderr():
    stream_fd = os.dup(sys.stdout.fileno())
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    return os.fdopen(stream_fd, "wb", buffering=0)


def label_text(image: Any, text: str, pos: tuple[int, int], color: tuple[int, int, int], scale: float = 0.75) -> None:
    black = (0, 0, 0)
    px, py = pos
    cv2.putText(image, text, (px + 2, py + 2), cv2.FONT_HERSHEY_SIMPLEX, scale, black, 4, cv2.LINE_AA)
    cv2.putText(image, text, (px, py), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def draw_roi_overlay(image: Any, roi: list[int]) -> None:
    x, y, w, h = roi
    blue = (255, 120, 40)
    yellow = (40, 230, 255)
    red = (40, 40, 255)
    green = (70, 210, 30)
    white = (255, 255, 255)

    cv2.rectangle(image, (x, y), (x + w, y + h), blue, 4)
    cv2.circle(image, (x, y), 7, yellow, -1)
    cv2.circle(image, (24, 24), 7, yellow, -1)
    cv2.arrowedLine(image, (24, 24), (230, 24), red, 5, tipLength=0.08)
    cv2.arrowedLine(image, (24, 24), (24, 230), green, 5, tipLength=0.08)
    label_text(image, "O (0,0)", (42, 46), yellow)
    label_text(image, "x", (240, 32), red, 0.9)
    label_text(image, "y", (36, 246), green, 0.9)
    label_text(image, "pixel: x -> right, y -> down", (70, 88), white, 0.7)
    label_text(image, f"ROI ({x},{y},{w},{h})", (x + 10, max(32, y - 16)), blue)


def draw_box_overlay(image: Any, box_pixels: dict[str, Any] | None) -> None:
    if not box_pixels:
        return
    import numpy as np

    try:
        points = np.asarray(box_pixels["box_px"], dtype=np.float32).reshape(4, 2)
    except (KeyError, TypeError, ValueError):
        return
    pts = np.round(points).astype(np.int32)
    cv2.polylines(image, [pts], True, (0, 200, 255), 4, cv2.LINE_AA)
    center = np.mean(points, axis=0)
    label_text(
        image,
        "box",
        (int(round(float(center[0]))) + 8, int(round(float(center[1]))) - 8),
        (0, 200, 255),
        0.7,
    )


def draw_seam_overlay(image: Any, seam_pixels: dict[str, Any] | None) -> None:
    if not seam_pixels:
        return
    lines = seam_pixels.get("lines")
    if isinstance(lines, list):
        colors = {
            "left_end": (0, 255, 255),
            "center": (0, 0, 255),
            "right_end": (255, 0, 255),
        }
        for line in lines:
            if not isinstance(line, dict):
                continue
            try:
                start = line["start_px"]
                end = line["end_px"]
                a = (int(round(float(start[0]))), int(round(float(start[1]))))
                b = (int(round(float(end[0]))), int(round(float(end[1]))))
            except (KeyError, TypeError, ValueError, IndexError):
                continue
            name = str(line.get("name", "cut_line"))
            color = colors.get(name, (0, 0, 255))
            cv2.line(image, a, b, color, 4, cv2.LINE_AA)
            cv2.circle(image, a, 7, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.circle(image, b, 7, (255, 0, 0), -1, cv2.LINE_AA)
            start_label = str(line.get("start_label") or "")
            end_label = str(line.get("end_label") or "")
            label_text(image, start_label or name, (a[0] + 10, a[1] - 10), (0, 255, 0), 0.85)
            label_text(image, end_label or name, (b[0] + 10, b[1] - 10), (255, 0, 0), 0.85)
        return

    try:
        start = seam_pixels["start_px"]
        end = seam_pixels["end_px"]
        a = (int(round(float(start[0]))), int(round(float(start[1]))))
        b = (int(round(float(end[0]))), int(round(float(end[1]))))
    except (KeyError, TypeError, ValueError, IndexError):
        return
    cv2.line(image, a, b, (0, 0, 255), 4, cv2.LINE_AA)
    cv2.circle(image, a, 7, (0, 255, 0), -1, cv2.LINE_AA)
    cv2.circle(image, b, 7, (255, 0, 0), -1, cv2.LINE_AA)
    label_text(image, "seam start", (a[0] + 10, a[1] - 10), (0, 255, 0), 0.65)
    label_text(image, "seam end", (b[0] + 10, b[1] - 10), (255, 0, 0), 0.65)


def parse_roi(value: str) -> list[int]:
    parts = [int(float(part.strip())) for part in value.split(",")]
    if len(parts) != 4:
        raise ValueError("roi must be x,y,w,h")
    return parts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--roi", default="400,150,400,400")
    parser.add_argument("--quality", type=int, default=58)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--show-roi", action="store_true")
    parser.add_argument("--show-box", action="store_true")
    parser.add_argument("--show-seam", action="store_true")
    parser.add_argument("--overlay-json", default="{}")
    parser.add_argument("--warmup-frames", type=int, default=5)
    parser.add_argument("--frame-timeout-ms", type=int, default=1500)
    args = parser.parse_args()

    out = binary_stdout_with_logs_on_stderr()
    roi = parse_roi(args.roi)
    quality = min(90, max(25, int(args.quality)))
    fps = min(30.0, max(1.0, float(args.fps)))
    frame_delay = 1.0 / fps
    overlays = json.loads(args.overlay_json or "{}")

    with args.config.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    for startup_attempt in range(1, 4):
        try:
            with OrbbecSDKCamera(
                serial_number=str(config["camera_serial"]),
                color_width=int(config.get("color_width", 1280)),
                color_height=int(config.get("color_height", 720)),
                fps=int(config.get("camera_fps", 30)),
                warmup_frames=max(0, int(args.warmup_frames)),
            ) as camera:
                missed_frames = 0
                while True:
                    started = time.time()
                    try:
                        frame = camera.wait_for_rgbd(timeout_ms=max(1, int(args.frame_timeout_ms)))
                    except RuntimeError as error:
                        missed_frames += 1
                        print(f"stream frame miss {missed_frames}: {error}", file=sys.stderr, flush=True)
                        if missed_frames >= 5:
                            raise
                        time.sleep(0.2)
                        continue
                    missed_frames = 0
                    image = frame.color_bgr.copy()
                    if args.show_roi:
                        draw_roi_overlay(image, roi)
                    if args.show_box:
                        draw_box_overlay(image, overlays.get("box_pixels"))
                    if args.show_seam:
                        draw_seam_overlay(image, overlays.get("seam_pixels"))
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        image,
                        [int(cv2.IMWRITE_JPEG_QUALITY), quality],
                    )
                    if not ok:
                        raise RuntimeError("failed to encode MJPEG frame")
                    payload = encoded.tobytes()
                    try:
                        out.write(f"--{BOUNDARY}\r\n".encode("ascii"))
                        out.write(b"Content-Type: image/jpeg\r\n")
                        out.write(f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii"))
                        out.write(payload)
                        out.write(b"\r\n")
                        out.flush()
                    except BrokenPipeError:
                        return 0
                    sleep_s = frame_delay - (time.time() - started)
                    if sleep_s > 0:
                        time.sleep(sleep_s)
        except BrokenPipeError:
            return 0
        except Exception as error:
            print(f"stream startup attempt {startup_attempt} failed: {error}", file=sys.stderr, flush=True)
            if startup_attempt >= 3:
                raise
            time.sleep(3.0)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
