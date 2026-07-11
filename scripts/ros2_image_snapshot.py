#!/usr/bin/env python3
"""Capture one ROS 2 image message and print a JSON data URL result."""
from __future__ import annotations

import argparse
import base64
import json
import re
import sys
import threading
import time
from io import BytesIO
from typing import Any

import numpy as np
from PIL import Image as PILImage

try:
    import rclpy
    from sensor_msgs.msg import CompressedImage, Image
except Exception as exc:  # pragma: no cover - depends on native ROS env
    print(json.dumps({"ok": False, "error": f"missing ROS 2 Python modules: {type(exc).__name__}: {exc}"}))
    sys.exit(2)


def _sanitize_node_name(topic: str) -> str:
    text = re.sub(r"[^a-zA-Z0-9_]+", "_", topic.strip("/")) or "image"
    return f"blacknode_image_snapshot_{text[:48]}"


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
    if channels is None:
        raise ValueError(
            f"unsupported encoding {encoding!r} "
            "(supported: mono8, rgb8, bgr8, rgba8, bgra8, 8UC1/3/4)"
        )

    step = int(msg.step or width * channels)
    if step < width * channels:
        raise ValueError(f"invalid step {step} for {width}x{height} {encoding}")
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


def _encode_data_url(image: PILImage.Image, output_format: str, jpeg_quality: int) -> tuple[str, str, int]:
    fmt = "JPEG" if output_format == "jpeg" else "PNG"
    mime = "image/jpeg" if fmt == "JPEG" else "image/png"
    img = image.convert("RGB") if fmt == "JPEG" else image
    buf = BytesIO()
    save_kwargs: dict[str, Any] = {}
    if fmt == "JPEG":
        save_kwargs["quality"] = max(1, min(100, int(jpeg_quality)))
    img.save(buf, format=fmt, **save_kwargs)
    data = buf.getvalue()
    return f"data:{mime};base64,{base64.b64encode(data).decode('ascii')}", mime, len(data)


def _metadata(msg: Any, message_type: str, pil: PILImage.Image, mime: str, byte_count: int) -> dict[str, Any]:
    base = {
        "width": int(pil.width),
        "height": int(pil.height),
        "mime_type": mime,
        "encoded_byte_count": int(byte_count),
        "stamp_sec": int(msg.header.stamp.sec),
        "stamp_nanosec": int(msg.header.stamp.nanosec),
        "frame_id": str(msg.header.frame_id),
    }
    if message_type == "compressed":
        base["format"] = str(getattr(msg, "format", ""))
        base["source_byte_count"] = len(bytes(msg.data))
    else:
        base["encoding"] = str(msg.encoding)
        base["step"] = int(msg.step)
        base["source_byte_count"] = len(bytes(msg.data))
    return base


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--message-type", choices=["raw", "compressed"], required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output-format", choices=["png", "jpeg"], default="png")
    parser.add_argument("--jpeg-quality", type=int, default=90)
    args = parser.parse_args()

    box: dict[str, Any] = {}
    event = threading.Event()

    rclpy.init(args=None)
    node = rclpy.create_node(_sanitize_node_name(args.topic))

    def callback(msg: Any) -> None:
        try:
            pil = _compressed_to_pil(msg) if args.message_type == "compressed" else _raw_image_to_pil(msg)
            image, mime, byte_count = _encode_data_url(pil, args.output_format, args.jpeg_quality)
            box["result"] = {
                "ok": True,
                "image": image,
                "metadata": _metadata(msg, args.message_type, pil, mime, byte_count),
            }
        except Exception as exc:
            box["result"] = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        finally:
            event.set()

    msg_cls = CompressedImage if args.message_type == "compressed" else Image
    node.create_subscription(msg_cls, args.topic, callback, 10)
    deadline = time.monotonic() + max(0.1, float(args.timeout))
    try:
        while rclpy.ok() and not event.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    result = box.get("result") or {"ok": False, "error": f"no image message on {args.topic} within {args.timeout:g}s"}
    print(json.dumps(result))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
