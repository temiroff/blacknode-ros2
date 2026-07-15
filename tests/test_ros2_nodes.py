"""blacknode-ros2 — node contracts.

The no-backend contract (structured error, never raises) is always exercised.
Integration tests run only when a real backend (native ros2 or Docker) is
available, and skip cleanly otherwise.
"""
import json
import math
import threading
import time
from pathlib import Path

import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_ros2 import ros2_runtime as rt
from blacknode.pkg.blacknode_ros2 import ros2_live as live
from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
from blacknode.pkg.blacknode_ros2 import rosbridge_service as service
from blacknode.workflow import validate_workflow

TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "templates"

EXPECTED_NODES = [
    "ROS2SystemCheck",
    "ROS2TopicList",
    "ROS2TopicEcho",
    "ROS2CompressedImageSnapshot",
    "ROS2ImageSnapshot",
    "ROS2ImageStream",
    "ROS2TopicPublish",
    "ROS2VisualDashboard",
    "ROS2DemoPublisher",
    "ROS2Launch",
    "ROS2Run",
    "ROS2NodeList",
    "ROS2ServiceList",
    "ROS2InterfaceShow",
    "ROS2PackageExecutables",
    "ROS2Command",
    "ROS2RosbridgeStatus",
    "ROS2RosbridgeServer",
    "ROS2RobotDiscovery",
    "ROS2JointState",
    "ROS2RotateJoint",
    "ROS2FollowDetectionJoint",
    "ROS2ContinuousFollowDetectionJoint",
    "ROS2NativeStatus",
    "ROS2NativeRobotDiscovery",
    "ROS2NativeJointState",
    "ROS2NativeSetJoint",
    "ROS2NativeFollowDetectionJoint",
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


def test_rosbridge_server_reuses_open_local_port(monkeypatch):
    monkeypatch.setattr(service, "_port_open", lambda host, port: True)
    monkeypatch.setattr(service, "_start_docker_desktop", lambda timeout: pytest.fail("Docker must not be touched"))

    result = _NODE_REGISTRY["ROS2RosbridgeServer"]({"action": "ensure"})

    assert result["ready"] is True
    assert "already running" in result["report"]


def test_rosbridge_server_reports_missing_docker(monkeypatch):
    monkeypatch.setattr(service, "_port_open", lambda host, port: False)
    monkeypatch.setattr(service, "_start_docker_desktop", lambda timeout: (_ for _ in ()).throw(RuntimeError("install Docker Desktop")))

    result = _NODE_REGISTRY["ROS2RosbridgeServer"]({"action": "ensure"})

    assert result["ready"] is False
    assert "install Docker Desktop" in result["report"]


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

    r = _NODE_REGISTRY["ROS2ImageSnapshot"]({"topic": "/camera/image_raw", "timeout": 1.0})
    assert r["image"] == ""
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2Launch"]({"package": "demo_nodes_cpp", "launch_file": "talker.launch.py"})
    assert r["launched"] is False
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2Run"]({"package": "demo_nodes_cpp", "executable": "talker"})
    assert r["running"] is False
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2ImageStream"]({"topic": "/camera/image_raw", "message_type": "raw"})
    assert r["preview"] == ""
    assert r["streaming"] is False
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


def test_compressed_image_snapshot_uses_snapshot_helper(monkeypatch):
    fake = {
        "ok": True,
        "backend": "native",
        "image": "data:image/jpeg;base64,/9j/2Q==",
        "metadata": {"width": 2, "height": 1, "format": "jpeg", "encoded_byte_count": 4},
    }
    captured = {}

    def fake_capture(**kwargs):
        captured.update(kwargs)
        return fake

    monkeypatch.setattr(rt, "capture_image_snapshot", fake_capture)
    result = _NODE_REGISTRY["ROS2CompressedImageSnapshot"]({
        "topic": "/camera/compressed",
        "timeout": 2.0,
    })
    assert result["image"].startswith("data:image/jpeg;base64,")
    assert result["metadata"]["width"] == 2
    assert captured["message_type"] == "compressed"
    assert captured["topic"] == "/camera/compressed"
    assert "captured compressed image frame" in result["report"]


def test_raw_image_snapshot_uses_snapshot_helper(monkeypatch):
    fake = {
        "ok": True,
        "backend": "native",
        "image": "data:image/png;base64,iVBORw0KGgo=",
        "metadata": {"width": 2, "height": 1, "encoding": "bgr8", "encoded_byte_count": 8},
    }
    captured = {}

    def fake_capture(**kwargs):
        captured.update(kwargs)
        return fake

    monkeypatch.setattr(rt, "capture_image_snapshot", fake_capture)
    result = _NODE_REGISTRY["ROS2ImageSnapshot"]({
        "topic": "/camera/image_raw",
        "timeout": 2.0,
        "output_format": "png",
    })
    assert result["image"].startswith("data:image/png;base64,")
    assert result["metadata"]["width"] == 2
    assert result["metadata"]["height"] == 1
    assert result["metadata"]["encoding"] == "bgr8"
    assert captured["message_type"] == "raw"
    assert captured["output_format"] == "png"
    assert "captured 2x1 bgr8 frame" in result["report"]


def test_launch_builds_ros2_launch_command(monkeypatch):
    captured = {}

    def fake_detached(args):
        captured["args"] = args
        return {"ok": True, "backend": "native"}

    monkeypatch.setattr(rt, "run_ros2_detached", fake_detached)
    result = _NODE_REGISTRY["ROS2Launch"]({
        "package": "camera_bringup",
        "launch_file": "camera.launch.py",
        "arguments": "device:=0 view:=false",
    })
    assert result["launched"] is True
    assert captured["args"] == [
        "launch",
        "camera_bringup",
        "camera.launch.py",
        "device:=0",
        "view:=false",
    ]
    assert "launch running" in result["report"]


def test_run_builds_ros2_run_command(monkeypatch):
    captured = {}

    def fake_managed(key, args):
        captured["key"] = key
        captured["args"] = args
        return {"ok": True, "backend": "native"}

    monkeypatch.setattr(rt, "run_ros2_managed", fake_managed)
    result = _NODE_REGISTRY["ROS2Run"]({
        "run_id": "camera_driver",
        "package": "demo_camera",
        "executable": "camera_node",
        "arguments": "--ros-args -r image:=/camera/image_raw",
    })
    assert result["running"] is True
    assert result["run_id"] == "camera_driver"
    assert captured["key"] == "camera_driver"
    assert captured["args"] == [
        "run",
        "demo_camera",
        "camera_node",
        "--ros-args",
        "-r",
        "image:=/camera/image_raw",
    ]
    assert "ROS 2 run process running" in result["report"]


def test_run_waits_for_expected_topic(monkeypatch):
    monkeypatch.setattr(rt, "run_ros2_managed", lambda key, args: {"ok": True, "backend": "native"})
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: {
        "ok": True,
        "backend": "native",
        "stdout": "/camera/image_raw\n/parameter_events\n",
        "stderr": "",
    })
    result = _NODE_REGISTRY["ROS2Run"]({
        "package": "demo_camera",
        "executable": "camera_node",
        "expected_topic": "/camera/image_raw",
        "wait_seconds": 1.0,
    })
    assert result["running"] is True
    assert "/camera/image_raw is discoverable" in result["report"]


def test_run_stop_calls_runtime(monkeypatch):
    captured = {}

    def fake_stop(key, pattern=""):
        captured["key"] = key
        captured["pattern"] = pattern
        return {"ok": True, "backend": "native", "stopped": 1}

    monkeypatch.setattr(rt, "stop_ros2_managed", fake_stop)
    result = _NODE_REGISTRY["ROS2Run"]({
        "action": "stop",
        "run_id": "camera_driver",
        "package": "demo_camera",
        "executable": "camera_node",
    })
    assert result["running"] is False
    assert captured == {"key": "camera_driver", "pattern": "ros2 run demo_camera camera_node"}
    assert "stopped 1" in result["report"]


def test_package_executables_lists_registered_commands(monkeypatch):
    fake = {
        "ok": True,
        "backend": "native",
        "stdout": "demo_camera camera_node\ndemo_camera calibration_panel\n",
        "stderr": "",
    }
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: fake)
    result = _NODE_REGISTRY["ROS2PackageExecutables"]({"package": "demo_camera"})
    assert result["executables"] == ["demo_camera camera_node", "demo_camera calibration_panel"]
    assert "OK" in result["report"]


def test_image_stream_starts_with_auto_raw_topic(monkeypatch):
    calls = {}

    def fake_run(args, timeout=15.0):
        assert args == ["topic", "type", "/camera/image_raw"]
        return {"ok": True, "backend": "native", "stdout": "sensor_msgs/msg/Image\n", "stderr": ""}

    def fake_start(**kwargs):
        calls.update(kwargs)
        return {
            "ok": True,
            "backend": "native",
            "stream_url": "http://127.0.0.1:9010/stream.mjpg",
            "snapshot_url": "http://127.0.0.1:9010/snapshot.jpg",
            "health_url": "http://127.0.0.1:9010/health.json",
            "port": 9010,
        }

    monkeypatch.setattr(rt, "run_ros2", fake_run)
    monkeypatch.setattr(rt, "start_image_stream", fake_start)
    result = _NODE_REGISTRY["ROS2ImageStream"]({
        "topic": "/camera/image_raw",
        "message_type": "auto",
        "stream_id": "cam",
        "max_fps": 12.0,
        "max_width": 800,
    })
    assert result["preview"] == "http://127.0.0.1:9010/stream.mjpg"
    assert result["streaming"] is True
    assert result["stream_url"] == result["preview"]
    assert calls["message_type"] == "raw"
    assert calls["topic"] == "/camera/image_raw"
    assert calls["stream_id"] == "cam"
    assert calls["max_fps"] == 12.0
    assert calls["max_width"] == 800


def test_image_stream_auto_detects_compressed_topic(monkeypatch):
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: {
        "ok": True,
        "backend": "native",
        "stdout": "sensor_msgs/msg/CompressedImage\n",
        "stderr": "",
    })
    monkeypatch.setattr(rt, "start_image_stream", lambda **kwargs: {
        "ok": True,
        "backend": "native",
        "stream_url": "http://127.0.0.1:9011/stream.mjpg",
        "snapshot_url": "http://127.0.0.1:9011/snapshot.jpg",
    })
    result = _NODE_REGISTRY["ROS2ImageStream"]({"topic": "/camera/compressed", "message_type": "auto"})
    assert result["preview"].endswith("/stream.mjpg")
    assert result["streaming"] is True
    assert "compressed" in result["report"]


def test_image_stream_stop_calls_runtime(monkeypatch):
    captured = {}

    def fake_stop(stream_id=""):
        captured["stream_id"] = stream_id
        return {"ok": True, "stopped": 1}

    monkeypatch.setattr(rt, "stop_image_stream", fake_stop)
    result = _NODE_REGISTRY["ROS2ImageStream"]({"action": "stop", "stream_id": "cam"})
    assert captured["stream_id"] == "cam"
    assert result["preview"] == ""
    assert result["streaming"] is False
    assert "stopped 1" in result["report"]


def test_docker_stream_port_allocator_uses_runtime_state(monkeypatch):
    class FakeProc:
        def poll(self):
            return None

    monkeypatch.setattr(rt, "STREAM_PORT_RANGE", "39000-39002")
    rt._streams.clear()
    rt._streams["a"] = {"backend": "docker", "port": 39000, "proc": FakeProc()}

    assert rt._free_docker_stream_port() == (39001, "")
    assert rt._free_docker_stream_port(39002) == (39002, "")
    assert rt._free_docker_stream_port(38999)[1].startswith("Docker ROS2ImageStream port must be within")


def test_runtime_stop_clears_streams_managed_runs_and_detached(monkeypatch):
    class FakeProc:
        pid = 12345

        def poll(self):
            return None

    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(rt, "_terminate_process", lambda proc: True)
    rt._streams.clear()
    rt._managed_detached.clear()
    rt._detached.clear()
    rt._streams["cam"] = {
        "proc": FakeProc(),
        "url": "http://127.0.0.1:9000/stream.mjpg",
        "snapshot_url": "http://127.0.0.1:9000/snapshot.jpg",
        "topic": "/camera/image_raw",
        "message_type": "raw",
    }
    rt._managed_detached["camera"] = FakeProc()
    rt._detached.append(FakeProc())

    result = rt.stop_runtime_services()

    assert result["ok"] is True
    assert result["stopped"] == {
        "streams": 1,
        "managed_runs": 1,
        "detached": 1,
        "continuous_follows": 0,
    }
    assert rt._streams == {}
    assert rt._managed_detached == {}
    assert rt._detached == []


# --- native rclpy robot control ---------------------------------------------------

def test_native_status_reports_topics(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "rclpy_version", lambda: "9.9.9")
    monkeypatch.setattr(nr, "topic_names_and_types", lambda timeout=1.0: [
        ("/joint_states", ["sensor_msgs/msg/JointState"]),
        ("/joint_commands", ["sensor_msgs/msg/JointState"]),
    ])
    monkeypatch.setattr(nr, "read_config", lambda *a, **k: {"commands_allowed": True})

    result = _NODE_REGISTRY["ROS2NativeStatus"]({})

    assert result["connected"] is True
    assert result["ready"] is True
    assert "/joint_states [sensor_msgs/msg/JointState]" in result["topics"]
    assert "rclpy:     OK" in result["report"]


def test_native_robot_discovery_reports_generic_profile(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_config", lambda *a, **k: {
        "commands_allowed": True,
        "joints": {"shoulder_pan": {"lower": math.radians(-90.0), "upper": math.radians(90.0)}},
    })
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: {
        "shoulder_pan": math.radians(10.0),
        "elbow": math.radians(25.0),
    })

    result = _NODE_REGISTRY["ROS2NativeRobotDiscovery"]({"units": "degrees"})

    assert result["connected"] is True
    assert result["ready"] is True
    assert result["robot"]["interface"]["kind"] == "native_ros2"
    assert result["robot"]["pose"]["shoulder_pan"] == 10.0
    assert result["robot"]["limits"]["shoulder_pan"]["lower"] == -90.0
    assert "=> READY" in result["report"]


def test_native_joint_state_reads_pose(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: {"gripper": math.radians(45.0)})

    result = _NODE_REGISTRY["ROS2NativeJointState"]({"units": "degrees"})

    assert result["pose"]["gripper"] == 45.0
    assert result["names"] == ["gripper"]
    assert "native rclpy" in result["report"]


def test_native_set_joint_previews_live_pose_when_disarmed(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("disarmed must never stream motion commands")

    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: {
        "shoulder_pan": math.radians(-11.6015625), "gripper": math.radians(25.0),
    })
    monkeypatch.setattr(nr, "stream_motion", fail_if_called)

    result = _NODE_REGISTRY["ROS2NativeSetJoint"]({
        "joint": "shoulder_pan",
        "position": 0.0,
        "units": "degrees",
        "armed": False,
    })

    assert result["moved"] is False
    assert result["report"].startswith("PREVIEW")
    assert math.isclose(result["before"]["shoulder_pan"], -11.6015625, abs_tol=1e-6)
    assert result["after"] == result["before"]
    assert math.isclose(result["target"]["shoulder_pan"], 0.0, abs_tol=1e-6)


def test_native_set_joint_streams_absolute_target(monkeypatch):
    start = {"shoulder_pan": 0.0, "gripper": math.radians(25.0)}
    after = {"shoulder_pan": 0.0, "gripper": math.radians(60.0)}
    poses = iter([start, after])
    captured = {}

    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: next(poses))

    def fake_stream(command_topic, names, s, t, **kwargs):
        captured["command_topic"] = command_topic
        captured["names"] = names
        captured["target"] = t
        return {"ok": True, "sent": 40}

    monkeypatch.setattr(nr, "stream_motion", fake_stream)

    result = _NODE_REGISTRY["ROS2NativeSetJoint"]({
        "joint": "gripper",
        "position": 60.0,
        "units": "degrees",
        "armed": True,
    })

    assert result["moved"] is True
    assert captured["command_topic"] == "/joint_commands"
    assert captured["names"] == ["shoulder_pan", "gripper"]
    assert math.isclose(captured["target"]["gripper"], math.radians(60.0), abs_tol=1e-6)
    assert "native set gripper" in result["report"]


def test_native_follow_detection_joint_blocked_when_disarmed(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("native ROS 2 must not be touched while disarmed")

    monkeypatch.setattr(nr, "read_pose", fail_if_called)
    monkeypatch.setattr(nr, "stream_motion", fail_if_called)

    result = _NODE_REGISTRY["ROS2NativeFollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "armed": False,
    })

    assert result["report"].startswith("BLOCKED:")
    assert result["moved"] is False
    assert result["command"] > 0


def test_native_follow_detection_joint_streams_toward_center(monkeypatch):
    start = {"shoulder_pan": math.radians(10.0), "elbow": 0.0}
    after = {"shoulder_pan": math.radians(20.0), "elbow": 0.0}
    poses = iter([start, after])
    captured = {}

    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: next(poses))

    def fake_stream(command_topic, names, s, t, **kwargs):
        captured["command_topic"] = command_topic
        captured["names"] = names
        captured["target"] = t
        return {"ok": True, "sent": 12}

    monkeypatch.setattr(nr, "stream_motion", fake_stream)

    result = _NODE_REGISTRY["ROS2NativeFollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "robot": {"state_topic": "/state", "command_topic": "/cmd"},
        "target_x": 0.5,
        "gain": 40.0,
        "max_step": 15.0,
        "units": "degrees",
        "armed": True,
    })

    assert result["moved"] is True
    assert math.isclose(result["command"], 10.0, abs_tol=1e-6)
    assert captured["command_topic"] == "/cmd"
    assert captured["names"] == ["shoulder_pan", "elbow"]
    assert math.isclose(captured["target"]["shoulder_pan"], math.radians(20.0), abs_tol=1e-6)
    assert "native follow shoulder_pan" in result["report"]


def test_native_nodes_structured_error_without_rclpy(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "rclpy is not importable"))

    status = _NODE_REGISTRY["ROS2NativeStatus"]({})
    assert status["ready"] is False
    assert "MISSING" in status["report"]

    state = _NODE_REGISTRY["ROS2NativeJointState"]({})
    assert state["pose"] == {}
    assert "rclpy is not importable" in state["report"]

    robot = _NODE_REGISTRY["ROS2NativeRobotDiscovery"]({})
    assert robot["ready"] is False
    assert "rclpy is not importable" in robot["robot"]["error"]

    set_joint = _NODE_REGISTRY["ROS2NativeSetJoint"]({"joint": "gripper", "armed": True})
    assert set_joint["moved"] is False
    assert "rclpy is not importable" in set_joint["report"]

    follow = _NODE_REGISTRY["ROS2NativeFollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "armed": True,
    })
    assert follow["moved"] is False
    assert "rclpy is not importable" in follow["report"]


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


def test_robot_discovery_reports_generic_profile(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "get_connection", lambda *a, **k: object())
    monkeypatch.setattr(rb, "read_config", lambda *a, **k: {
        "commands_allowed": True,
        "joints": {"shoulder_pan": {"lower": math.radians(-90.0), "upper": math.radians(90.0)}},
    })
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: {
        "shoulder_pan": math.radians(10.0),
        "elbow": math.radians(25.0),
    })

    result = _NODE_REGISTRY["ROS2RobotDiscovery"]({
        "host": "robot.local",
        "port": 9090,
        "units": "degrees",
    })

    assert result["connected"] is True
    assert result["ready"] is True
    assert result["joints"] == ["shoulder_pan", "elbow"]
    assert result["pose"]["shoulder_pan"] == 10.0
    assert result["robot"]["command_topic"] == "/joint_commands"
    assert result["robot"]["commands_allowed"] is True
    assert result["robot"]["limits"]["shoulder_pan"]["lower"] == -90.0
    assert "=> READY" in result["report"]


def test_robot_discovery_connection_failure_reports_diagnostics(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (True, ""))

    def fail_connect(*args, **kwargs):
        raise RuntimeError("Failed to connect to ROS")

    monkeypatch.setattr(rb, "get_connection", fail_connect)
    monkeypatch.setattr(live, "_rosbridge_connection_diagnostics", lambda host, port: [
        f"tcp port: closed at {host}:{port}",
        "local rosbridge_server: not found",
        "FIX: sudo apt install ros-jazzy-rosbridge-server",
    ])

    result = _NODE_REGISTRY["ROS2RobotDiscovery"]({})

    assert result["connected"] is False
    assert result["robot"]["diagnostics"][0] == "tcp port: closed at 127.0.0.1:9090"
    assert "local rosbridge_server: not found" in result["report"]
    assert "sudo apt install ros-jazzy-rosbridge-server" in result["report"]


def test_rosbridge_status_reports_connection_diagnostics(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (True, ""))

    def fail_connect(*args, **kwargs):
        raise RuntimeError("Failed to connect to ROS")

    monkeypatch.setattr(rb, "get_connection", fail_connect)
    monkeypatch.setattr(live, "_rosbridge_connection_diagnostics", lambda host, port: [
        f"tcp port: closed at {host}:{port}",
        "FIX: start it with: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090",
    ])

    result = _NODE_REGISTRY["ROS2RosbridgeStatus"]({})

    assert result["ready"] is False
    assert "UNREACHABLE" in result["report"]
    assert "tcp port: closed at 127.0.0.1:9090" in result["report"]
    assert "ros2 launch rosbridge_server" in result["report"]


def test_follow_detection_joint_blocked_when_disarmed(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("rosbridge must not be touched while disarmed")

    monkeypatch.setattr(rb, "get_connection", fail_if_called)
    monkeypatch.setattr(rb, "read_pose", fail_if_called)
    monkeypatch.setattr(rb, "stream_motion", fail_if_called)
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "armed": False,
    })
    assert result["report"].startswith("BLOCKED:")
    assert result["moved"] is False
    assert result["command"] > 0


def test_follow_detection_joint_streams_toward_center(monkeypatch):
    start = {"shoulder_pan": math.radians(10.0), "elbow": 0.0}
    after = {"shoulder_pan": math.radians(20.0), "elbow": 0.0}
    poses = iter([start, after])
    captured = {}

    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: next(poses))

    def fake_stream(host, port, command_topic, names, s, t, **kwargs):
        captured["host"] = host
        captured["command_topic"] = command_topic
        captured["names"] = names
        captured["target"] = t
        return {"ok": True, "sent": 12}

    monkeypatch.setattr(rb, "stream_motion", fake_stream)
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 160}},
        "frame_width": 640,
        "robot": {"host": "robot.local", "port": 9090, "state_topic": "/state", "command_topic": "/cmd"},
        "target_x": 0.5,
        "gain": 40.0,
        "max_step": 15.0,
        "units": "degrees",
        "armed": True,
    })
    assert result["moved"] is True
    assert math.isclose(result["command"], 10.0, abs_tol=1e-6)
    assert captured["host"] == "robot.local"
    assert captured["command_topic"] == "/cmd"
    assert math.isclose(captured["target"]["shoulder_pan"], math.radians(20.0), abs_tol=1e-6)
    assert captured["names"] == ["shoulder_pan", "elbow"]
    assert "cube zone=LEFT, x=160.0/640" in result["report"]


def test_follow_detection_uses_payload_frame_width_and_reports_zone(monkeypatch):
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: pytest.fail("disarmed preview must not read ROS"))
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 455}, "frame_width": 640},
        "frame_width": 960,
        "target_x": 0.5,
        "deadband": 0.16,
        "gain": 10.0,
        "max_step": 2.0,
        "invert": True,
        "armed": False,
    })

    assert result["command"] == 2.0
    assert "zone=RIGHT, x=455.0/640" in result["report"]


def test_follow_detection_joint_noops_inside_deadband(monkeypatch):
    def fail_if_streamed(*args, **kwargs):
        raise AssertionError("must not stream commands inside deadband")

    monkeypatch.setattr(rb, "stream_motion", fail_if_streamed)
    result = _NODE_REGISTRY["ROS2FollowDetectionJoint"]({
        "joint": "shoulder_pan",
        "detection": {"found": True, "center": {"x": 322}},
        "frame_width": 640,
        "target_x": 0.5,
        "deadband": 0.02,
        "armed": True,
    })
    assert result["moved"] is False
    assert result["command"] == 0.0
    assert "centered enough" in result["report"]


def test_continuous_follow_runs_until_stopped(monkeypatch):
    called = threading.Event()

    def fake_follow(item, ctx):
        called.set()
        return {
            "moved": True,
            "joint": ctx["joint"],
            "before": {ctx["joint"]: 0.0},
            "after": {ctx["joint"]: 2.0},
            "target": {ctx["joint"]: 2.0},
            "error": 0.25,
            "command": 2.0,
            "report": "MOVE LEFT",
        }

    monkeypatch.setattr(live, "_continuous_follow_step", fake_follow)
    live.stop_continuous_follow_services()
    try:
        started = _NODE_REGISTRY["ROS2ContinuousFollowDetectionJoint"]({
            "action": "start",
            "run_id": "test_follow",
            "loop_hz": 20.0,
            "detection_url": "http://127.0.0.1:9999/detection.json",
            "joint": "shoulder_pan",
            "armed": True,
        })
        assert started["running"] is True
        assert called.wait(1.0)

        checked = _NODE_REGISTRY["ROS2ContinuousFollowDetectionJoint"]({
            "action": "check",
            "run_id": "test_follow",
            "joint": "shoulder_pan",
        })
        assert checked["running"] is True
        assert checked["command"] == 2.0

        stopped = _NODE_REGISTRY["ROS2ContinuousFollowDetectionJoint"]({
            "action": "stop",
            "run_id": "test_follow",
            "joint": "shoulder_pan",
        })
        assert stopped["running"] is False
        assert "stopped" in stopped["report"]
        assert live.continuous_follow_runtime_status() == []
    finally:
        live.stop_continuous_follow_services()


def test_continuous_follow_step_reuses_persistent_joint_stream(monkeypatch):
    class FakeSession:
        def __init__(self):
            self.published = []

        def snapshot(self):
            return ({"shoulder_pan": 0.0, "elbow": 0.25}, {}, 0.01)

        def wait_for_pose(self, timeout):
            return {"shoulder_pan": 0.0, "elbow": 0.25}

        def publish(self, pose):
            self.published.append(pose)

    session = FakeSession()
    acquired = []
    monkeypatch.setattr(live, "_read_detection_url", lambda *a, **k: ({
        "found": True,
        "updated_at": time.time(),
        "detection": {"found": True, "center": {"x": 100}, "frame_width": 640},
    }, ""))
    monkeypatch.setattr(rb, "acquire_joint_stream", lambda *a, **k: acquired.append((a, k)) or session)
    monkeypatch.setattr(rb, "release_joint_stream", lambda _session: None)
    item = {"session": None, "session_signature": None}
    ctx = {
        "joint": "shoulder_pan",
        "units": "degrees",
        "detection_stream": {"url": "http://detector/detection.json", "stream_id": "cube"},
        "loop_hz": 10.0,
        "gain": 10.0,
        "max_step": 2.0,
        "armed": True,
    }

    first = live._continuous_follow_step(item, ctx)
    second = live._continuous_follow_step(item, ctx)

    assert first["running"] is True
    assert second["running"] is True
    assert len(acquired) == 1
    assert len(session.published) == 2
    assert set(session.published[0]) == {"shoulder_pan", "elbow"}
    assert session.published[1]["shoulder_pan"] > session.published[0]["shoulder_pan"]


def test_continuous_follow_step_resets_stale_joint_stream(monkeypatch):
    class FakeSession:
        def snapshot(self):
            return ({"shoulder_pan": 0.0}, {}, 99.0)

        def wait_for_pose(self, timeout):
            return {"shoulder_pan": 0.0}

    session = FakeSession()
    released = []
    monkeypatch.setattr(live, "_read_detection_url", lambda *a, **k: ({
        "found": True,
        "updated_at": time.time(),
        "detection": {"found": True, "center": {"x": 100}, "frame_width": 640},
    }, ""))
    monkeypatch.setattr(rb, "release_joint_stream", released.append)
    signature = ("127.0.0.1", 9090, "/joint_states", "/joint_commands", "")
    item = {"session": session, "session_signature": signature, "session_resets": 0}
    ctx = {
        "joint": "shoulder_pan",
        "units": "degrees",
        "detection_stream": {"url": "http://detector/detection.json", "stream_id": "cube"},
        "loop_hz": 10.0,
        "gain": 10.0,
        "max_step": 2.0,
        "armed": True,
    }

    result = live._continuous_follow_step(item, ctx)

    assert result["running"] is False
    assert "resetting subscription" in result["report"]
    assert released == [session]
    assert item["session"] is None
    assert item["session_signature"] is None
    assert item["session_resets"] == 1


def test_continuous_follow_disarmed_does_not_start():
    live.stop_continuous_follow_services()
    result = _NODE_REGISTRY["ROS2ContinuousFollowDetectionJoint"]({
        "action": "start",
        "run_id": "test_disarmed",
        "detection_url": "http://127.0.0.1:9999/detection.json",
        "joint": "shoulder_pan",
        "armed": False,
    })
    assert result["running"] is False
    assert result["report"].startswith("BLOCKED:")
    assert live.continuous_follow_runtime_status() == []


def test_live_nodes_structured_error_without_roslibpy(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (False, "roslibpy is not installed"))
    status = _NODE_REGISTRY["ROS2RosbridgeStatus"]({})
    assert status["ready"] is False
    assert "MISSING" in status["report"]

    live = _NODE_REGISTRY["ROS2JointState"]({})
    assert live["pose"] == {}
    assert "FAILED" in live["report"]

    robot = _NODE_REGISTRY["ROS2RobotDiscovery"]({})
    assert robot["ready"] is False
    assert "roslibpy is not installed" in robot["robot"]["error"]


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
