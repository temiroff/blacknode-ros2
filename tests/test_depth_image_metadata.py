"""Depth metadata emitted by the generic ROS 2 image helpers."""
import runpy
import struct
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace


_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def _load_helper(monkeypatch, filename):
    rclpy = ModuleType("rclpy")
    qos = ModuleType("rclpy.qos")
    qos.qos_profile_sensor_data = object()
    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msg_types = ModuleType("sensor_msgs.msg")
    sensor_msg_types.Image = type("Image", (), {})
    sensor_msg_types.CompressedImage = type("CompressedImage", (), {})
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    monkeypatch.setitem(sys.modules, "sensor_msgs", sensor_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msg_types)
    return runpy.run_path(str(_SCRIPTS / filename), run_name="depth_helper_test")


def _depth_message():
    return SimpleNamespace(
        height=2,
        width=2,
        encoding="16UC1",
        step=4,
        is_bigendian=False,
        data=struct.pack("<HHHH", 100, 500, 1000, 0),
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=34),
            frame_id="depth_frame",
        ),
    )


def test_stream_and_snapshot_helpers_publish_bounded_raw_depth_summary(monkeypatch):
    for filename in (
        "ros2_image_stream_server.py",
        "ros2_image_snapshot.py",
    ):
        module = _load_helper(monkeypatch, filename)
        summary = module["_raw_depth_summary"](_depth_message())

        assert summary["encoding"] == "16UC1"
        assert summary["valid_count"] == 3
        assert summary["total_count"] == 4
        assert summary["minimum"] == 100.0
        assert 100.0 <= summary["p05"] <= 500.0
        assert summary["median"] == 500.0


def test_snapshot_metadata_marks_receive_time_and_depth_summary(monkeypatch):
    module = _load_helper(monkeypatch, "ros2_image_snapshot.py")
    metadata = module["_metadata"](
        _depth_message(),
        "raw",
        SimpleNamespace(width=2, height=2, mode="L"),
        "image/jpeg",
        42,
    )

    assert metadata["received_at_ns"] > 0
    assert metadata["depth_summary_raw"]["valid_count"] == 3


def test_stream_helper_packs_metric_depth_without_json_expanding_pixels(monkeypatch):
    module = _load_helper(monkeypatch, "ros2_image_stream_server.py")
    payload = module["_metric_depth_frame"](_depth_message())

    assert payload.startswith(b"BNDEPTH1")
    header_size = struct.unpack("<I", payload[8:12])[0]
    header = json.loads(payload[12:12 + header_size])
    pixels = payload[12 + header_size:]

    assert header["kind"] == "blacknode.metric-depth-frame"
    assert header["width"] == 2
    assert header["height"] == 2
    assert header["encoding"] == "16UC1"
    assert pixels == struct.pack("<HHHH", 100, 500, 1000, 0)


def test_stream_helper_renders_fixed_metric_range_and_marks_invalid_pixels(monkeypatch):
    module = _load_helper(monkeypatch, "ros2_image_stream_server.py")
    payload = module["_metric_depth_frame"](_depth_message())

    image = module["_metric_depth_preview"](
        payload,
        depth_scale=0.001,
        auto_range=False,
        near_m=0.1,
        far_m=1.0,
        palette="grayscale",
        invalid_color="magenta",
    )

    assert image.mode == "RGB"
    assert image.getpixel((0, 0)) == (255, 255, 255)
    middle = image.getpixel((1, 0))
    assert 140 <= middle[0] <= 142
    assert middle[0] == middle[1] == middle[2]
    assert image.getpixel((0, 1)) == (0, 0, 0)
    assert image.getpixel((1, 1)) == (255, 0, 255)
