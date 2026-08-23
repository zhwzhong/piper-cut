#!/usr/bin/env python3
"""Small OrbbecSDK RGB-D adapter used by the ROS-free calibration tools."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    K: list[float]
    D: list[float]
    distortion_model: str = "plumb_bob"


@dataclass
class RGBDFrame:
    color_bgr: np.ndarray
    depth_mm: np.ndarray
    color_timestamp_us: float
    depth_timestamp_us: float
    intrinsics: CameraIntrinsics


def _frame_to_bgr(frame) -> np.ndarray:
    from pyorbbecsdk import OBFormat

    width = frame.get_width()
    height = frame.get_height()
    data = np.asanyarray(frame.get_data())
    color_format = frame.get_format()
    if color_format == OBFormat.RGB:
        image = np.resize(data, (height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if color_format == OBFormat.BGR:
        return np.resize(data, (height, width, 3)).copy()
    if color_format == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("OpenCV could not decode the Orbbec MJPG color frame")
        return image
    if color_format == OBFormat.YUYV:
        image = np.resize(data, (height, width, 2))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
    if color_format == OBFormat.UYVY:
        image = np.resize(data, (height, width, 2))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
    if color_format == OBFormat.I420:
        image = np.resize(data, (height * 3 // 2, width))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    if color_format == OBFormat.NV12:
        image = np.resize(data, (height * 3 // 2, width))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
    if color_format == OBFormat.NV21:
        image = np.resize(data, (height * 3 // 2, width))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV21)
    raise RuntimeError(f"unsupported Orbbec color format: {color_format}")


def _profile_items(profile_list):
    if hasattr(profile_list, "get_count"):
        count = int(profile_list.get_count())
        for index in range(count):
            profile = profile_list.get_stream_profile_by_index(index)
            if hasattr(profile, "is_video_stream_profile") and profile.is_video_stream_profile():
                profile = profile.as_video_stream_profile()
            yield profile
    else:
        for profile in profile_list:
            yield profile


def _choose_video_profile(
    profile_list,
    width: Optional[int],
    height: Optional[int],
    fps: Optional[int],
    preferred_formats: tuple,
):
    profiles = [
        profile
        for profile in _profile_items(profile_list)
        if all(hasattr(profile, name) for name in ("get_width", "get_height", "get_fps"))
    ]
    if not profiles:
        raise RuntimeError("Orbbec returned no video stream profiles")

    def score(profile) -> tuple:
        exact_size = width is None or height is None or (
            profile.get_width() == width and profile.get_height() == height
        )
        exact_fps = fps is None or profile.get_fps() == fps
        try:
            format_rank = preferred_formats.index(profile.get_format())
        except ValueError:
            format_rank = len(preferred_formats)
        size_distance = 0
        if width is not None and height is not None:
            size_distance = abs(profile.get_width() - width) + abs(profile.get_height() - height)
        fps_distance = 0 if fps is None else abs(profile.get_fps() - fps)
        return (not exact_size, not exact_fps, format_rank, size_distance, fps_distance)

    chosen = min(profiles, key=score)
    if width is not None and height is not None:
        if chosen.get_width() != width or chosen.get_height() != height:
            raise RuntimeError(
                f"requested {width}x{height}, closest profile is "
                f"{chosen.get_width()}x{chosen.get_height()}"
            )
    return chosen


class OrbbecSDKCamera:
    def __init__(
        self,
        serial_number: Optional[str] = None,
        color_width: int = 1280,
        color_height: int = 720,
        depth_width: Optional[int] = None,
        depth_height: Optional[int] = None,
        fps: int = 30,
        warmup_frames: int = 15,
    ) -> None:
        from pyorbbecsdk import (
            AlignFilter,
            Config,
            Context,
            OBFormat,
            OBFrameAggregateOutputMode,
            OBSensorType,
            OBStreamType,
            Pipeline,
        )

        self._context = Context()
        devices = self._context.query_devices()
        matches = []
        for index in range(devices.get_count()):
            device = devices.get_device_by_index(index)
            info = device.get_device_info()
            if serial_number is None or info.get_serial_number() == serial_number:
                matches.append(device)
        if not matches:
            raise RuntimeError(f"Orbbec camera serial {serial_number!r} was not found")
        if serial_number is None and len(matches) != 1:
            serials = [item.get_device_info().get_serial_number() for item in matches]
            raise RuntimeError(f"multiple Orbbec cameras found; select one by serial: {serials}")

        self._device = matches[0]
        info = self._device.get_device_info()
        self.device_name = str(info.get_name())
        self.serial_number = str(info.get_serial_number())
        self.connection_type = str(info.get_connection_type())
        self._pipeline = Pipeline(self._device)
        config = Config()
        color_profiles = self._pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = _choose_video_profile(
            color_profiles,
            color_width,
            color_height,
            fps,
            (OBFormat.RGB, OBFormat.MJPG, OBFormat.YUYV),
        )
        depth_profiles = self._pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        if depth_width is None or depth_height is None:
            depth_profile = depth_profiles.get_default_video_stream_profile()
        else:
            depth_profile = _choose_video_profile(
                depth_profiles,
                depth_width,
                depth_height,
                fps,
                (OBFormat.Y16,),
            )
        config.enable_stream(color_profile)
        config.enable_stream(depth_profile)
        config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        try:
            self._pipeline.enable_frame_sync()
        except Exception:
            pass
        self._align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self.color_profile = {
            "width": int(color_profile.get_width()),
            "height": int(color_profile.get_height()),
            "fps": int(color_profile.get_fps()),
            "format": str(color_profile.get_format()),
        }
        self.depth_profile = {
            "width": int(depth_profile.get_width()),
            "height": int(depth_profile.get_height()),
            "fps": int(depth_profile.get_fps()),
            "format": str(depth_profile.get_format()),
        }
        self._pipeline.start(config)
        self._closed = False
        for _ in range(max(0, int(warmup_frames))):
            self._pipeline.wait_for_frames(1000)

    @staticmethod
    def _intrinsics(color_frame) -> CameraIntrinsics:
        profile = color_frame.get_stream_profile()
        if hasattr(profile, "as_video_stream_profile"):
            profile = profile.as_video_stream_profile()
        intrinsic = profile.get_intrinsic()
        distortion = profile.get_distortion()
        K = [
            float(intrinsic.fx),
            0.0,
            float(intrinsic.cx),
            0.0,
            float(intrinsic.fy),
            float(intrinsic.cy),
            0.0,
            0.0,
            1.0,
        ]
        D = [
            float(getattr(distortion, "k1", 0.0)),
            float(getattr(distortion, "k2", 0.0)),
            float(getattr(distortion, "p1", 0.0)),
            float(getattr(distortion, "p2", 0.0)),
            float(getattr(distortion, "k3", 0.0)),
            float(getattr(distortion, "k4", 0.0)),
            float(getattr(distortion, "k5", 0.0)),
            float(getattr(distortion, "k6", 0.0)),
        ]
        return CameraIntrinsics(
            width=int(color_frame.get_width()),
            height=int(color_frame.get_height()),
            K=K,
            D=D,
        )

    def wait_for_rgbd(self, timeout_ms: int = 1500) -> RGBDFrame:
        frames = self._pipeline.wait_for_frames(timeout_ms)
        if frames is None:
            raise RuntimeError("timed out waiting for Orbbec RGB-D frames")
        aligned = self._align.process(frames)
        if aligned is None:
            raise RuntimeError("Orbbec depth-to-color alignment returned no frames")
        if hasattr(aligned, "as_frame_set"):
            aligned = aligned.as_frame_set()
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if color_frame is None or depth_frame is None:
            raise RuntimeError("aligned Orbbec frame set is missing color or depth")
        color = _frame_to_bgr(color_frame)
        raw_depth = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        expected = depth_frame.get_width() * depth_frame.get_height()
        if raw_depth.size != expected:
            raise RuntimeError(
                f"depth payload has {raw_depth.size} pixels, expected {expected}"
            )
        depth_scaled = raw_depth.reshape(
            depth_frame.get_height(), depth_frame.get_width()
        ).astype(np.float32) * float(depth_frame.get_depth_scale())
        depth_mm = np.clip(np.rint(depth_scaled), 0, np.iinfo(np.uint16).max).astype(np.uint16)
        if color.shape[:2] != depth_mm.shape:
            raise RuntimeError(
                f"aligned RGB-D sizes differ: color={color.shape[:2]}, depth={depth_mm.shape}"
            )
        color_ts = (
            color_frame.get_timestamp_us()
            if hasattr(color_frame, "get_timestamp_us")
            else float(color_frame.get_timestamp()) * 1000.0
        )
        depth_ts = (
            depth_frame.get_timestamp_us()
            if hasattr(depth_frame, "get_timestamp_us")
            else float(depth_frame.get_timestamp()) * 1000.0
        )
        return RGBDFrame(
            color_bgr=color,
            depth_mm=depth_mm,
            color_timestamp_us=float(color_ts),
            depth_timestamp_us=float(depth_ts),
            intrinsics=self._intrinsics(color_frame),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._pipeline.stop()

    def __enter__(self) -> "OrbbecSDKCamera":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
