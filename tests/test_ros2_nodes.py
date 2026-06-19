"""blacknode-ros2 — node contracts.

The no-backend contract (structured error, never raises) is always exercised.
Integration tests run only when a real backend (native ros2 or Docker) is
available, and skip cleanly otherwise.
"""
import json
import math
from pathlib import Path

import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_ros2 import ros2_runtime as rt
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
from blacknode.workflow import validate_workflow

TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "templates"

EXPECTED_NODES = [
    "ROS2SystemCheck",
    "ROS2TopicList",
    "ROS2TopicEcho",
    "ROS2CompressedImageSnapshot",
    "ROS2TopicPublish",
    "ROS2VisualDashboard",
    "ROS2DemoPublisher",
    "ROS2NodeList",
    "ROS2ServiceList",
    "ROS2InterfaceShow",
    "ROS2Command",
    "ROS2RosbridgeStatus",
    "ROS2JointState",
    "ROS2RotateJoint",
    "ROS2MotionDashboard",
]

HAS_BACKEND = rt.detect_backend()["backend"] != "none"
backend_only = pytest.mark.skipif(not HAS_BACKEND, reason="no ros2 CLI and no Docker daemon")


# --- contracts that always hold ---------------------------------------------------

def test_all_nodes_registered_with_category():
    for name in EXPECTED_NODES:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "ROS 2"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-ros2"


def test_templates_validate():
    for path in sorted(TEMPLATE_DIR.glob("*.json")):
        report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
        assert report.ok, f"{path.name}: {report.to_dict()}"


def test_visual_dashboard_reports_roundtrip_pass():
    result = _NODE_REGISTRY["ROS2VisualDashboard"]({
        "status": "backend: docker (ros:jazzy)\nros2 CLI reachable: yes",
        "publisher": "demo publisher running on /blacknode_demo at 5 Hz via docker",
        "echo_report": "received 1 message(s)",
        "messages": ["data: Blacknode ROS 2 roundtrip works"],
        "topics": ["/blacknode_demo [std_msgs/msg/String]"],
        "nodes": [],
        "services": [],
        "definition": "string data",
    })
    assert result["passed"] is True
    assert result["summary"]["topic_ok"] is True
    assert result["dashboard"].startswith("data:image/svg+xml;base64,")


def test_no_backend_is_structured_error(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "none", "detail": "x"})
    r = _NODE_REGISTRY["ROS2TopicList"]({"show_types": True})
    assert r["topics"] == []
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2TopicEcho"]({"topic": "/chatter"})
    assert r["messages"] == []
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2DemoPublisher"]({"action": "start"})
    assert "FAILED" in r["report"]


def test_system_check_reports_unavailable(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "none", "detail": "no ros"})
    r = _NODE_REGISTRY["ROS2SystemCheck"]({"refresh": True})
    assert r["available"] is False
    assert r["backend"] == "none"


def test_command_rejects_empty_args():
    r = _NODE_REGISTRY["ROS2Command"]({"args": "  "})
    assert "FAILED" in r["report"]


def test_echo_keeps_partial_messages_on_timeout(monkeypatch):
    fake = {
        "ok": False, "timed_out": True, "backend": "docker",
        "stdout": "data: a\n---\ndata: b\n---", "stderr": "", "error": "timed out",
    }
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: fake)
    r = _NODE_REGISTRY["ROS2TopicEcho"]({"topic": "/chatter", "count": 5})
    assert len(r["messages"]) == 2
    assert "received 2" in r["report"]


def test_compressed_image_snapshot_decodes_ros_yaml(monkeypatch):
    fake = {
        "ok": True,
        "backend": "native",
        "stdout": "header: {}\nformat: jpeg\ndata: [255, 216, 255, 217]\n---",
        "stderr": "",
    }
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: fake)
    result = _NODE_REGISTRY["ROS2CompressedImageSnapshot"]({
        "topic": "/camera/compressed",
        "timeout": 2.0,
    })
    assert result["image"].startswith("data:image/jpeg;base64,")
    assert result["metadata"]["byte_count"] == 4
    assert "captured 4 byte" in result["report"]


# --- universal live robot control over rosbridge ----------------------------------

def test_rotate_joint_needs_a_joint_name():
    result = _NODE_REGISTRY["ROS2RotateJoint"]({"joint": "", "armed": True})
    assert result["report"].startswith("BLOCKED:")
    assert result["moved"] is False


def test_rotate_joint_blocked_when_disarmed(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("rosbridge must not be touched while disarmed")

    monkeypatch.setattr(rb, "get_connection", fail_if_called)
    monkeypatch.setattr(rb, "read_pose", fail_if_called)
    monkeypatch.setattr(rb, "stream_motion", fail_if_called)
    result = _NODE_REGISTRY["ROS2RotateJoint"]({"joint": "gripper", "armed": False})
    assert result["report"].startswith("BLOCKED:")
    assert result["moved"] is False


def test_rotate_joint_refuses_read_only_bridge(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_config", lambda *a, **k: {"commands_allowed": False, "joints": {}})

    def fail_if_streamed(*args, **kwargs):
        raise AssertionError("must not stream commands to a read-only bridge")

    monkeypatch.setattr(rb, "stream_motion", fail_if_streamed)
    result = _NODE_REGISTRY["ROS2RotateJoint"]({
        "joint": "gripper", "armed": True, "config_topic": "/joint_config",
    })
    assert result["report"].startswith("BLOCKED:")
    assert "read-only" in result["report"]


def test_rotate_joint_streams_and_reports_motion(monkeypatch):
    # values in radians on the wire; node is asked for degrees
    start = {"shoulder_pan": 0.0, "gripper": math.radians(25.0)}
    after = {"shoulder_pan": 0.0, "gripper": math.radians(60.0)}
    poses = iter([start, after])
    captured = {}

    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: next(poses))

    def fake_stream(host, port, command_topic, names, s, t, **kwargs):
        captured["start"] = s
        captured["target"] = t
        return {"ok": True, "sent": 100}

    monkeypatch.setattr(rb, "stream_motion", fake_stream)
    result = _NODE_REGISTRY["ROS2RotateJoint"]({
        "joint": "gripper", "delta": 35.0, "units": "degrees", "armed": True,
    })
    assert result["moved"] is True
    assert math.isclose(captured["target"]["gripper"], math.radians(60.0), abs_tol=1e-6)
    assert result["before"]["gripper"] == 25.0
    assert "25.00 -> 60.00 degrees" in result["report"]


def test_rotate_joint_clamps_to_config_limits(monkeypatch):
    start = {"gripper": math.radians(90.0)}
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_config", lambda *a, **k: {
        "commands_allowed": True,
        "joints": {"gripper": {"lower": 0.0, "upper": math.radians(100.0)}},
    })
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: dict(start))
    monkeypatch.setattr(rb, "stream_motion", lambda *a, **k: {"ok": True, "sent": 10})
    result = _NODE_REGISTRY["ROS2RotateJoint"]({
        "joint": "gripper", "delta": 35.0, "units": "degrees", "armed": True,
        "config_topic": "/joint_config",
    })
    assert math.isclose(result["target"]["gripper"], 100.0, abs_tol=1e-6)
    assert "clamped from 125.0" in result["report"]


def test_live_nodes_structured_error_without_roslibpy(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (False, "roslibpy is not installed"))
    status = _NODE_REGISTRY["ROS2RosbridgeStatus"]({})
    assert status["ready"] is False
    assert "MISSING" in status["report"]

    live = _NODE_REGISTRY["ROS2JointState"]({})
    assert live["pose"] == {}
    assert "FAILED" in live["report"]


def test_motion_dashboard_renders_before_after():
    before = {"shoulder_pan": 0.0, "gripper": 25.0}
    after = {"shoulder_pan": 0.0, "gripper": 60.0}
    result = _NODE_REGISTRY["ROS2MotionDashboard"]({
        "joint": "gripper",
        "before": before,
        "after": after,
        "target": {"gripper": 60.0},
        "moved": True,
        "units": "degrees",
    })
    assert result["dashboard"].startswith("data:image/svg+xml;base64,")
    assert result["summary"]["delta"] == 35.0
    assert result["summary"]["moved"] is True


# --- integration (needs native ros2 or Docker) ------------------------------------

@backend_only
def test_system_check_live():
    r = _NODE_REGISTRY["ROS2SystemCheck"]({"refresh": True})
    assert r["available"] is True, r["report"]
    assert r["backend"] in ("native", "docker")


@backend_only
def test_publish_then_echo_roundtrip():
    start = _NODE_REGISTRY["ROS2DemoPublisher"](
        {"action": "start", "topic": "/bn_test", "message": "roundtrip", "rate": 5.0}
    )
    assert "FAILED" not in start["report"], start["report"]
    try:
        r = _NODE_REGISTRY["ROS2TopicEcho"]({"topic": "/bn_test", "count": 1, "timeout": 30.0})
        assert r["messages"], r["report"]
        assert "roundtrip" in r["messages"][0]

        topics = _NODE_REGISTRY["ROS2TopicList"]({"show_types": False})
        assert any("/bn_test" in t for t in topics["topics"]), topics
    finally:
        _NODE_REGISTRY["ROS2DemoPublisher"]({"action": "stop"})


@backend_only
def test_interface_show_live():
    r = _NODE_REGISTRY["ROS2InterfaceShow"]({"interface": "std_msgs/msg/String"})
    assert "string data" in r["definition"], r["report"]
