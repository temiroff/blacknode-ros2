"""blacknode-ros2 — node contracts.

The no-backend contract (structured error, never raises) is always exercised.
Integration tests run only when a real backend (native ros2 or Docker) is
available, and skip cleanly otherwise.
"""
import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_ros2 import ros2_runtime as rt

EXPECTED_NODES = [
    "ROS2SystemCheck",
    "ROS2TopicList",
    "ROS2TopicEcho",
    "ROS2TopicPublish",
    "ROS2DemoPublisher",
    "ROS2NodeList",
    "ROS2ServiceList",
    "ROS2InterfaceShow",
    "ROS2Command",
]

HAS_BACKEND = rt.detect_backend()["backend"] != "none"
backend_only = pytest.mark.skipif(not HAS_BACKEND, reason="no ros2 CLI and no Docker daemon")


# --- contracts that always hold ---------------------------------------------------

def test_all_nodes_registered_with_category():
    for name in EXPECTED_NODES:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "ROS 2"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-ros2"


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
