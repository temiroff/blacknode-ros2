#!/usr/bin/env python3
"""Publish frames from an MJPEG source onto a ROS 2 image topic.

This helper is launched through ros2_runtime.start_host_camera_publisher(),
which the blacknode-perception camera ROS 2 adapter drives (ROS2USBCamera). It
exists because a ROS 2 graph running inside the Docker helper container cannot
open a host USB webcam: Docker Desktop gives the container no ``/dev/video*``.
Blacknode's own Camera node can open that webcam on the host and serve it as
MJPEG, so this bridges that MJPEG stream into the ROS graph as
``sensor_msgs/msg/Image``.

Like the stream server it avoids cv_bridge, using NumPy and Pillow instead so
it runs in the same environment as the other helpers.
"""
from __future__ import annotations

import argparse
import re
import signal
import sys
import threading
import time
import urllib.request
from io import BytesIO

import numpy as np
from PIL import Image as PILImage

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
except Exception as exc:  # pragma: no cover - depends on native ROS env
    print(f"missing ROS 2 Python modules: {type(exc).__name__}: {exc}", file=sys.stderr)
    sys.exit(2)

_BOUNDARY_RE = re.compile(rb"--[^\r\n]+\r\n")
_CONTENT_LENGTH_RE = re.compile(rb"Content-Length:\s*(\d+)", re.IGNORECASE)


def _sanitize_node_name(topic: str) -> str:
    text = re.sub(r"[^a-zA-Z0-9_]+", "_", topic.strip("/")) or "camera"
    return f"blacknode_host_camera_{text[:48]}"


def _iter_mjpeg_frames(url: str, stop: threading.Event, timeout: float):
    """Yield JPEG payloads from a multipart/x-mixed-replace MJPEG stream."""
    response = urllib.request.urlopen(url, timeout=timeout)
    buffer = b""
    try:
        while not stop.is_set():
            chunk = response.read(8192)
            if not chunk:
                return
            buffer += chunk
            # Frames are "--boundary\r\nheaders\r\n\r\n<jpeg bytes>"; use the
            # declared Content-Length so a JPEG containing boundary-like bytes
            # cannot truncate the frame.
            while True:
                header_end = buffer.find(b"\r\n\r\n")
                if header_end < 0:
                    break
                header = buffer[:header_end]
                match = _CONTENT_LENGTH_RE.search(header)
                if not match:
                    # no length header: drop up to the next boundary and retry
                    nxt = _BOUNDARY_RE.search(buffer, header_end)
                    if not nxt:
                        break
                    buffer = buffer[nxt.end():]
                    continue
                length = int(match.group(1))
                start = header_end + 4
                if len(buffer) < start + length:
                    break
                yield buffer[start:start + length]
                buffer = buffer[start + length:]
    finally:
        try:
            response.close()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-url", required=True, help="MJPEG stream URL to read")
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--frame-id", default="camera_frame")
    parser.add_argument("--max-fps", type=float, default=15.0)
    parser.add_argument("--connect-timeout", type=float, default=15.0)
    args = parser.parse_args()

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, lambda *_: stop.set())
        except Exception:
            pass

    rclpy.init(args=None)
    node = rclpy.create_node(_sanitize_node_name(args.topic))
    publisher = node.create_publisher(Image, args.topic, qos_profile_sensor_data)

    min_interval = 1.0 / max(0.1, float(args.max_fps))
    last_sent = 0.0
    published = 0

    # Reconnect rather than exit: the host camera stream may still be starting,
    # or may restart while this bridge is running.
    while not stop.is_set():
        try:
            for jpeg in _iter_mjpeg_frames(args.source_url, stop, args.connect_timeout):
                if stop.is_set():
                    break
                now = time.monotonic()
                if now - last_sent < min_interval:
                    continue
                last_sent = now
                try:
                    frame = PILImage.open(BytesIO(jpeg)).convert("RGB")
                except Exception:
                    continue
                array = np.asarray(frame, dtype=np.uint8)
                height, width = array.shape[0], array.shape[1]
                message = Image()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = str(args.frame_id)
                message.height = int(height)
                message.width = int(width)
                message.encoding = "rgb8"
                message.is_bigendian = 0
                message.step = int(width * 3)
                message.data = array.tobytes()
                publisher.publish(message)
                published += 1
                if published % 60 == 1:
                    print(f"published {published} frame(s) to {args.topic}", flush=True)
        except Exception as exc:
            if stop.is_set():
                break
            print(f"host camera source error: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        if stop.wait(1.0):
            break

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
