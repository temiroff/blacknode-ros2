#!/usr/bin/env python3
"""Serve a ROS 2 image topic as a local MJPEG stream.

This helper is launched by the generic Blacknode ROS2 node for image topics. It intentionally
does not depend on cv_bridge so it can handle common raw encodings with NumPy
and Pillow in the Blacknode environment.
"""
from __future__ import annotations

import argparse
import json
import re
import signal
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from typing import Any
from urllib.parse import parse_qs, urlsplit

import numpy as np
from PIL import Image as PILImage
from PIL import ImageDraw

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
except Exception as exc:  # pragma: no cover - depends on native ROS env
    print(f"missing ROS 2 Python modules: {type(exc).__name__}: {exc}", file=sys.stderr)
    sys.exit(2)


class FrameStore:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.metric_frame: bytes | None = None
        self.metadata: dict[str, Any] = {}
        self.frame_count = 0
        self.error = ""
        self.last_encoded_at = 0.0

    def put(
        self,
        jpeg: bytes,
        metadata: dict[str, Any],
        metric_frame: bytes | None = None,
    ) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.metric_frame = metric_frame
            self.metadata = metadata
            self.frame_count += 1
            self.error = ""
            self.condition.notify_all()

    def fail(self, message: str) -> None:
        with self.condition:
            self.error = message
            self.condition.notify_all()


def _sanitize_node_name(topic: str) -> str:
    text = re.sub(r"[^a-zA-Z0-9_]+", "_", topic.strip("/")) or "image"
    return f"blacknode_image_stream_{text[:48]}"


def _raw_depth_summary(msg: Image) -> dict[str, Any]:
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding or "").strip().lower()
    if height <= 0 or width <= 0 or encoding not in {
        "mono16",
        "16uc1",
        "32fc1",
    }:
        return {}
    bytes_per_pixel = 4 if encoding == "32fc1" else 2
    step = int(msg.step or width * bytes_per_pixel)
    if step < width * bytes_per_pixel:
        return {}
    raw = bytes(msg.data)
    required = height * step
    if len(raw) < required:
        return {}
    dtype = np.dtype(
        (">f4" if bool(msg.is_bigendian) else "<f4")
        if encoding == "32fc1"
        else (">u2" if bool(msg.is_bigendian) else "<u2")
    )
    rows = np.frombuffer(raw[:required], dtype=np.uint8).reshape((height, step))
    packed = rows[:, : width * bytes_per_pixel].copy()
    depth = packed.view(dtype).reshape((height, width)).astype(np.float32)
    valid = depth[np.isfinite(depth) & (depth > 0)]
    if valid.size == 0:
        return {
            "encoding": str(msg.encoding),
            "valid_count": 0,
            "total_count": int(width * height),
        }
    minimum, p05, median, p95 = np.percentile(
        valid,
        [0.0, 5.0, 50.0, 95.0],
    )
    return {
        "encoding": str(msg.encoding),
        "valid_count": int(valid.size),
        "total_count": int(width * height),
        "minimum": float(minimum),
        "p05": float(p05),
        "median": float(median),
        "p95": float(p95),
    }


def _metric_depth_frame(msg: Image) -> bytes | None:
    """Pack one metric depth image as bounded binary plus a small JSON header."""
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding or "").strip()
    normalized_encoding = encoding.lower()
    if height <= 0 or width <= 0 or normalized_encoding not in {
        "mono16",
        "16uc1",
        "32fc1",
    }:
        return None
    bytes_per_pixel = 4 if normalized_encoding == "32fc1" else 2
    step = int(msg.step or width * bytes_per_pixel)
    if step < width * bytes_per_pixel:
        return None
    raw = bytes(msg.data)
    required = height * step
    if required <= 0 or required > 256 * 1024 * 1024 or len(raw) < required:
        return None
    header = json.dumps(
        {
            "kind": "blacknode.metric-depth-frame",
            "schema_version": 1,
            "width": width,
            "height": height,
            "step": step,
            "encoding": encoding,
            "is_bigendian": bool(msg.is_bigendian),
            "frame_id": str(msg.header.frame_id),
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "received_at_ns": time.time_ns(),
        },
        separators=(",", ":"),
    ).encode("utf-8")
    return b"BNDEPTH1" + struct.pack("<I", len(header)) + header + raw[:required]


def _depth_array_to_pil(
    depth: np.ndarray,
    *,
    near: float | None = None,
    far: float | None = None,
    palette: str = "grayscale",
    invalid_color: str = "black",
) -> PILImage.Image:
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        if invalid_color == "magenta":
            pixels = np.empty((*depth.shape, 3), dtype=np.uint8)
            pixels[:, :] = (255, 0, 255)
            return PILImage.fromarray(pixels)
        return PILImage.fromarray(np.zeros(depth.shape, dtype=np.uint8))

    if near is None or far is None:
        near, far = np.percentile(depth[valid], [2.0, 98.0])
    near = float(near)
    far = float(far)
    if far <= near:
        far = near + 1.0
    normalized = np.clip((depth - near) / (far - near), 0.0, 1.0)
    intensity = np.where(valid, (1.0 - normalized) * 255.0, 0.0).astype(
        np.uint8
    )
    if palette == "grayscale" and invalid_color == "black":
        return PILImage.fromarray(intensity)

    x = intensity.astype(np.float32) / 255.0
    if palette == "turbo":
        coefficients = (
            (0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943),
            (0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604),
            (0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973),
        )
        powers = np.stack([np.ones_like(x), x, x**2, x**3, x**4, x**5])
        rgb = np.stack(
            [np.clip(np.tensordot(row, powers, axes=(0, 0)), 0.0, 1.0) for row in coefficients],
            axis=-1,
        )
        pixels = (rgb * 255.0).astype(np.uint8)
    else:
        pixels = np.repeat(intensity[:, :, None], 3, axis=2)
    if invalid_color == "magenta":
        pixels[~valid] = (255, 0, 255)
    else:
        pixels[~valid] = (0, 0, 0)
    return PILImage.fromarray(pixels)


def _metric_depth_preview(
    payload: bytes,
    *,
    depth_scale: float,
    auto_range: bool,
    near_m: float,
    far_m: float,
    palette: str,
    invalid_color: str,
) -> PILImage.Image:
    if not payload.startswith(b"BNDEPTH1") or len(payload) < 12:
        raise ValueError("invalid metric depth frame")
    header_size = struct.unpack("<I", payload[8:12])[0]
    header_end = 12 + header_size
    if header_end > len(payload):
        raise ValueError("metric depth frame header is truncated")
    header = json.loads(payload[12:header_end].decode("utf-8"))
    height = int(header.get("height") or 0)
    width = int(header.get("width") or 0)
    encoding = str(header.get("encoding") or "").strip().lower()
    bytes_per_pixel = 4 if encoding == "32fc1" else 2
    if height <= 0 or width <= 0 or encoding not in {"mono16", "16uc1", "32fc1"}:
        raise ValueError("metric depth frame has unsupported dimensions or encoding")
    step = int(header.get("step") or width * bytes_per_pixel)
    raw = payload[header_end:]
    required = height * step
    if step < width * bytes_per_pixel or len(raw) < required:
        raise ValueError("metric depth frame pixels are truncated")
    dtype = np.dtype(
        (">f4" if bool(header.get("is_bigendian")) else "<f4")
        if encoding == "32fc1"
        else (">u2" if bool(header.get("is_bigendian")) else "<u2")
    )
    rows = np.frombuffer(raw[:required], dtype=np.uint8).reshape((height, step))
    packed = rows[:, : width * bytes_per_pixel].copy()
    depth_m = packed.view(dtype).reshape((height, width)).astype(np.float32)
    depth_m *= max(0.0, float(depth_scale))
    return _depth_array_to_pil(
        depth_m,
        near=None if auto_range else near_m,
        far=None if auto_range else far_m,
        palette=palette,
        invalid_color=invalid_color,
    )


def _raw_image_to_pil(msg: Image) -> PILImage.Image:
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding or "").strip().lower()
    if height <= 0 or width <= 0:
        raise ValueError("image message had invalid height/width")

    channels_by_encoding = {
        "mono8": 1,
        "8uc1": 1,
        "rgb8": 3,
        "bgr8": 3,
        "8uc3": 3,
        "rgba8": 4,
        "bgra8": 4,
        "8uc4": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None and encoding in {"mono16", "16uc1", "32fc1"}:
        bytes_per_pixel = 4 if encoding == "32fc1" else 2
        step = int(msg.step or width * bytes_per_pixel)
        raw = bytes(msg.data)
        required = height * step
        if len(raw) < required:
            raise ValueError(
                f"image data too short: {len(raw)} bytes, expected {required}"
            )
        dtype = np.dtype(
            (">f4" if bool(msg.is_bigendian) else "<f4")
            if encoding == "32fc1"
            else (">u2" if bool(msg.is_bigendian) else "<u2")
        )
        rows = np.frombuffer(raw[:required], dtype=np.uint8).reshape((height, step))
        packed = rows[:, : width * bytes_per_pixel].copy()
        depth = packed.view(dtype).reshape((height, width)).astype(np.float32)
        return _depth_array_to_pil(depth)
    if channels is None:
        raise ValueError(f"unsupported encoding {encoding!r}")

    step = int(msg.step or width * channels)
    raw = bytes(msg.data)
    required = height * step
    if len(raw) < required:
        raise ValueError(f"image data too short: {len(raw)} bytes, expected {required}")

    rows = np.frombuffer(raw[:required], dtype=np.uint8).reshape((height, step))
    if channels == 1:
        pixels = rows[:, :width].copy()
    else:
        pixels = rows[:, : width * channels].reshape((height, width, channels)).copy()
        if encoding == "bgr8":
            pixels = pixels[:, :, [2, 1, 0]]
        elif encoding == "bgra8":
            pixels = pixels[:, :, [2, 1, 0, 3]]
    return PILImage.fromarray(pixels)


def _compressed_to_pil(msg: CompressedImage) -> PILImage.Image:
    return PILImage.open(BytesIO(bytes(msg.data)))


def _draw_badge(img: PILImage.Image, text: str, subtext: str = "") -> PILImage.Image:
    draw = ImageDraw.Draw(img)
    pad = 8
    line_h = 18
    width = max(96, int(max(draw.textlength(text), draw.textlength(subtext)) if subtext else draw.textlength(text)) + 22)
    height = 32 if not subtext else 54
    draw.rounded_rectangle((pad, pad, pad + width, pad + height), radius=8, fill=(13, 17, 28), outline=(239, 68, 68), width=2)
    draw.ellipse((pad + 10, pad + 11, pad + 20, pad + 21), fill=(239, 68, 68))
    draw.text((pad + 28, pad + 8), text, fill=(255, 255, 255))
    if subtext:
        draw.text((pad + 12, pad + 8 + line_h), subtext, fill=(190, 203, 220))
    return img


def _placeholder_jpeg(store: FrameStore, args: argparse.Namespace, *, quality: int) -> bytes:
    width = max(360, int(args.max_width) if int(args.max_width) > 0 else 960)
    height = max(220, int(width * 9 / 16))
    img = PILImage.new("RGB", (width, height), (11, 16, 32))
    draw = ImageDraw.Draw(img)
    with store.condition:
        frames = store.frame_count
        error = store.error
    title = "LIVE STREAM"
    detail = error or "waiting for ROS image frames"
    lines = [
        title,
        f"topic: {args.topic}",
        f"type: {args.message_type}",
        f"frames: {frames}",
        detail,
    ]
    y = max(26, height // 2 - 58)
    for i, line in enumerate(lines):
        fill = (248, 250, 252) if i == 0 else (147, 164, 184)
        draw.text((28, y + i * 24), line, fill=fill)
    _draw_badge(img, "LIVE", "stream server running")
    return _encode_jpeg(img, quality)


def _encode_jpeg(img: PILImage.Image, quality: int) -> bytes:
    buf = BytesIO()
    img.save(buf, format="JPEG", quality=max(1, min(100, quality)))
    return buf.getvalue()


def _jpeg_bytes(image: PILImage.Image, *, max_width: int, quality: int, topic: str) -> bytes:
    img = image.convert("RGB")
    if max_width > 0 and img.width > max_width:
        height = max(1, int(img.height * (max_width / float(img.width))))
        img = img.resize((max_width, height), PILImage.Resampling.LANCZOS)
    # No badge on real frames: a moving picture already shows it is live, and
    # burning a label into the pixels also corrupts the frame for anything
    # downstream that treats this stream as camera data.
    return _encode_jpeg(img, quality)


def _preview_frame(
    jpeg: bytes | None,
    metric_frame: bytes | None,
    query: dict[str, list[str]],
    args: argparse.Namespace,
) -> bytes | None:
    if not metric_frame or not any(key.startswith("depth_") for key in query):
        return jpeg
    try:
        mode = str(query.get("depth_range", ["auto"])[0]).strip().lower()
        depth_scale = float(query.get("depth_scale", ["1.0"])[0])
        near_m = float(query.get("depth_near_m", ["0.2"])[0])
        far_m = float(query.get("depth_far_m", ["2.0"])[0])
        palette = str(query.get("depth_palette", ["grayscale"])[0]).strip().lower()
        invalid_color = str(query.get("depth_invalid", ["black"])[0]).strip().lower()
        if palette not in {"grayscale", "turbo"}:
            palette = "grayscale"
        if invalid_color not in {"black", "magenta"}:
            invalid_color = "black"
        if depth_scale <= 0.0 or near_m < 0.0 or far_m <= near_m:
            return jpeg
        image = _metric_depth_preview(
            metric_frame,
            depth_scale=depth_scale,
            auto_range=mode != "fixed",
            near_m=near_m,
            far_m=far_m,
            palette=palette,
            invalid_color=invalid_color,
        )
        return _jpeg_bytes(
            image,
            max_width=int(args.max_width),
            quality=int(args.jpeg_quality),
            topic=args.topic,
        )
    except (TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError, struct.error):
        return jpeg


def _make_handler(store: FrameStore, stop_event: threading.Event, args: argparse.Namespace):
    class Handler(BaseHTTPRequestHandler):
        server_version = "BlacknodeImageStream/0.1"

        def log_message(self, fmt: str, *values: Any) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802 - stdlib API
            request = urlsplit(self.path)
            query = parse_qs(request.query)
            if request.path in ("/", "/index.html"):
                self._index()
            elif request.path == "/stream.mjpg":
                self._stream(query)
            elif request.path == "/snapshot.jpg":
                self._snapshot(query)
            elif request.path == "/health.json":
                self._health()
            elif request.path == "/frame.bin":
                self._metric_frame()
            else:
                self.send_error(404)

        def _index(self) -> None:
            body = (
                "<!doctype html><title>Blacknode ROS 2 Stream</title>"
                "<style>html,body{margin:0;background:#111;height:100%;display:grid;place-items:center}"
                "img{max-width:100vw;max-height:100vh}</style>"
                '<img src="/stream.mjpg" alt="ROS 2 image stream">'
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _health(self) -> None:
            with store.condition:
                payload = {
                    "topic": args.topic,
                    "message_type": args.message_type,
                    "frames": store.frame_count,
                    "metadata": store.metadata,
                    "error": store.error,
                }
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _snapshot(self, query: dict[str, list[str]]) -> None:
            with store.condition:
                frame = store.jpeg
                metric_frame = store.metric_frame
            frame = _preview_frame(frame, metric_frame, query, args)
            if not frame:
                frame = _placeholder_jpeg(store, args, quality=int(args.jpeg_quality))
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

        def _metric_frame(self) -> None:
            with store.condition:
                frame = store.metric_frame
            if not frame:
                self.send_error(503, "metric depth frame unavailable")
                return
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "application/vnd.blacknode.metric-depth-frame",
            )
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)

        def _stream(self, query: dict[str, list[str]]) -> None:
            boundary = "blacknode-frame"
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            last_count = -1
            last_placeholder = 0.0
            while not stop_event.is_set():
                with store.condition:
                    store.condition.wait(timeout=0.5)
                    frame = store.jpeg
                    metric_frame = store.metric_frame
                    count = store.frame_count
                now = time.monotonic()
                if not frame:
                    if now - last_placeholder < 1.0:
                        continue
                    frame = _placeholder_jpeg(store, args, quality=int(args.jpeg_quality))
                    last_placeholder = now
                elif count == last_count:
                    continue
                else:
                    last_count = count
                    frame = _preview_frame(frame, metric_frame, query, args)
                try:
                    self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break

    return Handler


def _spin_ros(store: FrameStore, stop_event: threading.Event, args: argparse.Namespace) -> None:
    rclpy.init(args=None)
    node = rclpy.create_node(_sanitize_node_name(args.topic))
    min_period = 1.0 / max(0.1, float(args.max_fps))

    def handle_image(msg: Any) -> None:
        now = time.monotonic()
        if now - store.last_encoded_at < min_period:
            return
        store.last_encoded_at = now
        try:
            if args.message_type == "compressed":
                pil = _compressed_to_pil(msg)
                metadata = {
                    "format": str(getattr(msg, "format", "")),
                    "stamp_sec": int(msg.header.stamp.sec),
                    "stamp_nanosec": int(msg.header.stamp.nanosec),
                    "frame_id": str(msg.header.frame_id),
                }
            else:
                pil = _raw_image_to_pil(msg)
                metadata = {
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": str(msg.encoding),
                    "stamp_sec": int(msg.header.stamp.sec),
                    "stamp_nanosec": int(msg.header.stamp.nanosec),
                    "frame_id": str(msg.header.frame_id),
                }
                depth_summary = _raw_depth_summary(msg)
                if depth_summary:
                    metadata["depth_summary_raw"] = depth_summary
            metadata["received_at_ns"] = time.time_ns()
            store.put(
                _jpeg_bytes(
                    pil,
                    max_width=int(args.max_width),
                    quality=int(args.jpeg_quality),
                    topic=args.topic,
                ),
                metadata,
                _metric_depth_frame(msg) if args.message_type == "raw" else None,
            )
        except Exception as exc:  # keep stream process alive for later valid frames
            store.fail(f"{type(exc).__name__}: {exc}")

    msg_cls = CompressedImage if args.message_type == "compressed" else Image
    # Camera drivers (image_tools cam2image, usb_cam, v4l2_camera, ...)
    # conventionally publish images with best-effort QoS. A subscriber on the
    # default (reliable) QoS can never connect to a best-effort publisher --
    # it looks "subscribed" but silently receives zero frames forever. Match
    # the sensor-data QoS preset so this works against any compliant driver.
    node.create_subscription(msg_cls, args.topic, handle_image, qos_profile_sensor_data)
    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--message-type", choices=["raw", "compressed"], required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--max-fps", type=float, default=10.0)
    parser.add_argument("--max-width", type=int, default=960)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    args = parser.parse_args()

    stop_event = threading.Event()
    store = FrameStore()

    def stop(*_: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    ros_thread = threading.Thread(target=_spin_ros, args=(store, stop_event, args), daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer((args.host, args.port), _make_handler(store, stop_event, args))
    server.timeout = 0.5
    try:
        while not stop_event.is_set():
            server.handle_request()
    finally:
        server.server_close()
        stop_event.set()
        ros_thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
