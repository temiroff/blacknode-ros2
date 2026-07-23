"""blacknode-ros2 — integration primitive contracts.

Graph discovery, topics, services, processes, and the native/rosbridge
transports. Capability nodes built on these (joint control, camera streaming)
live in their own packages' ROS 2 adapters and are tested there.

The no-backend contract (structured error, never raises) is always exercised.
Integration tests run only when a real backend (native ros2 or Docker) is
available, and skip cleanly otherwise.
"""
import json
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import (
    _PACKAGE_REGISTRY,
    component_dependency_plan,
)
from blacknode.pkg.blacknode_ros2 import ros2_runtime as rt
from blacknode.pkg.blacknode_ros2 import ros2_live as live
from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
from blacknode.pkg.blacknode_ros2 import rosbridge_service as service
from blacknode.workflow import validate_workflow

TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "templates"
PACKAGE_DIR = TEMPLATE_DIR.parent

EXPECTED_NODES = [
    "ROS2BridgeEcho",
    "ROS2BridgePublish",
    "ROS2InterfaceShow",
    "ROS2Launch",
    "ROS2NodeList",
    "ROS2PackageExecutables",
    "ROS2RosbridgeServer",
    "ROS2RosbridgeStatus",
    "ROS2Run",
    "ROS2ServiceList",
    "ROS2Status",
    "ROS2SystemCheck",
    "ROS2TopicEcho",
    "ROS2TopicList",
    "ROS2TopicPublish",
    "ROS2TopicPublisher",
    "ROS2VisualDashboard",
]

EXPECTED_COMPONENT_NODES = {
    "core": set(),
    "rosbridge": {
        "ROS2BridgeEcho",
        "ROS2BridgePublish",
        "ROS2RosbridgeServer",
        "ROS2RosbridgeStatus",
    },
    "topics": {
        "ROS2TopicEcho",
        "ROS2TopicList",
        "ROS2TopicPublish",
        "ROS2TopicPublisher",
    },
    "services": {"ROS2ServiceList"},
    "processes": {"ROS2Launch", "ROS2PackageExecutables", "ROS2Run"},
    "diagnostics": {
        "ROS2InterfaceShow",
        "ROS2NodeList",
        "ROS2Status",
        "ROS2SystemCheck",
        "ROS2VisualDashboard",
    },
}

HAS_BACKEND = rt.detect_backend()["backend"] != "none"
backend_only = pytest.mark.skipif(not HAS_BACKEND, reason="no ros2 CLI and no Docker daemon")


# --- contracts that always hold ---------------------------------------------------

def test_all_nodes_registered_with_category():
    for name in EXPECTED_NODES:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "ROS 2"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-ros2"


def test_components_own_their_registration_paths_and_depend_on_core():
    info = _PACKAGE_REGISTRY["blacknode-ros2"]

    assert set(info.components) == set(EXPECTED_COMPONENT_NODES)
    assert info.components["core"]["node_paths"] == ["nodes"]
    assert info.components["core"]["module_root"] is True

    for component_name, expected_nodes in EXPECTED_COMPONENT_NODES.items():
        component = info.components[component_name]
        registered = {
            name for name, fn in _NODE_REGISTRY.items()
            if getattr(fn, "_bn_package", "") == "blacknode-ros2"
            and getattr(fn, "_bn_component", "") == component_name
        }

        assert set(component["node_types"]) == expected_nodes
        assert registered == expected_nodes
        if component_name == "core":
            assert component["requirements"] == []
            continue

        expected_path = f"components/{component_name}/nodes"
        assert component["node_paths"] == [expected_path]
        assert component["requirements"] == [{
            "package": "",
            "component": "core",
            "version": "",
        }]
        for node_name in expected_nodes:
            source = str(_NODE_REGISTRY[node_name]._bn_source_path).replace("\\", "/")
            assert source.endswith(expected_path), (node_name, source)


def test_component_dependency_plan_enables_core_first():
    plan = component_dependency_plan("blacknode-ros2", "topics")

    assert [
        (item["package"], item["component"])
        for item in plan["plan"]
    ] == [
        ("blacknode-ros2", "core"),
        ("blacknode-ros2", "topics"),
    ]


def test_disabled_component_does_not_register_its_nodes(tmp_path):
    probe_name = "blacknode-ros2-component-probe"
    probe_dir = tmp_path / probe_name
    shutil.copytree(
        PACKAGE_DIR,
        probe_dir,
        ignore=shutil.ignore_patterns(".git", ".pytest_cache", "__pycache__"),
    )
    manifest_path = probe_dir / "blacknode-package.toml"
    manifest = manifest_path.read_text(encoding="utf-8")
    manifest_path.write_text(
        manifest.replace(
            'name = "blacknode-ros2"',
            f'name = "{probe_name}"',
            1,
        ),
        encoding="utf-8",
    )
    (tmp_path / ".blacknode-components.json").write_text(
        json.dumps({
            "schema_version": 1,
            "packages": {probe_name: {"topics": False}},
        }),
        encoding="utf-8",
    )

    expected_nodes = sorted(set(EXPECTED_NODES) - EXPECTED_COMPONENT_NODES["topics"])
    script = f"""
from blacknode.packages import load_package
info = load_package(r"{probe_dir}")
assert info.ok, info.error
assert "topics" not in info.enabled_components
assert sorted(info.node_types) == {expected_nodes!r}
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_topic_publisher_has_generic_contract():
    publisher = _NODE_REGISTRY["ROS2TopicPublisher"]

    assert "ROS2DemoPublisher" not in _NODE_REGISTRY
    assert publisher._bn_inputs == [
        "trigger", "action", "topic", "msg_type", "payload", "rate_hz",
    ]
    assert publisher._bn_outputs == ["running", "backend", "report"]
    assert publisher._bn_hidden is False
    assert _NODE_REGISTRY["ROS2VisualDashboard"]._bn_hidden is True


def test_capability_nodes_are_not_owned_by_the_integration_layer():
    """Camera and joint-control nodes belong to their capability packages.

    They may be registered (those packages are installed too), but never by
    this one -- that is what keeps the ROS 2 layer free of domain verticals.
    """
    for name in [
        "CameraROS2Subscribe", "CameraROS2Publish", "CameraROS2Http",
        "ROS2JointState", "ROS2SetJoint", "ROS2ManualMove", "ROS2MotionDashboard",
    ]:
        owner = getattr(_NODE_REGISTRY.get(name), "_bn_package", "")
        assert owner != "blacknode-ros2", f"{name} is still owned by blacknode-ros2"


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


def test_generic_status_prefers_native_when_rclpy_is_available(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(live, "ros2_native_status", lambda ctx: {
        "connected": True, "ready": True, "topics": ["/joint_states"], "config": {}, "report": "native ready",
    })
    monkeypatch.setattr(service, "ros2_rosbridge_server", lambda ctx: pytest.fail("rosbridge must not start"))

    result = _NODE_REGISTRY["ROS2Status"]({"transport": "auto"})

    assert result["transport"] == "native"
    assert result["ready"] is True
    assert "auto-selected" in result["report"]


def test_generic_status_falls_back_to_rosbridge_and_ensures_service(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "missing rclpy"))
    monkeypatch.setattr(service, "ros2_rosbridge_server", lambda ctx: {"ready": True, "report": "server ready"})
    monkeypatch.setattr(live, "ros2_rosbridge_status", lambda ctx: {
        "connected": True, "ready": True, "config": {}, "report": "bridge ready",
    })

    result = _NODE_REGISTRY["ROS2Status"]({"transport": "auto", "ensure_rosbridge": True})

    assert result["transport"] == "rosbridge"
    assert result["ready"] is True
    assert "bridge ready" in result["report"]


def test_templates_validate():
    for path in sorted(TEMPLATE_DIR.glob("*.json")):
        report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
        assert report.ok, f"{path.name}: {report.to_dict()}"


def test_templates_declare_exact_component_requirements():
    expected = {
        "ros2-connect-robot-wifi.json": {
            "blacknode-ros2/core",
            "blacknode-ros2/rosbridge",
        },
        "ros2-publish-subscribe.json": {
            "blacknode-ros2/core",
            "blacknode-ros2/topics",
            "blacknode-ros2/services",
            "blacknode-ros2/diagnostics",
        },
        "ros2-run-your-package.json": {
            "blacknode-ros2/core",
            "blacknode-ros2/topics",
            "blacknode-ros2/processes",
            "blacknode-ros2/diagnostics",
        },
    }
    for path in sorted(TEMPLATE_DIR.glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        assert set(workflow["metadata"]["required_components"]) == expected[path.name]


def test_visual_dashboard_reports_roundtrip_pass():
    result = _NODE_REGISTRY["ROS2VisualDashboard"]({
        "status": "backend: docker (ros:jazzy)\nros2 CLI reachable: yes",
        "publisher": "topic publisher running on /blacknode_demo at 5 Hz via docker",
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

    r = _NODE_REGISTRY["ROS2TopicPublisher"]({"action": "start"})
    assert r["running"] is False
    assert r["backend"] == "none"
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2Launch"]({"package": "demo_nodes_cpp", "launch_file": "talker.launch.py"})
    assert r["launched"] is False
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2Run"]({"package": "demo_nodes_cpp", "executable": "talker"})
    assert r["running"] is False
    assert "FAILED" in r["report"]


def test_system_check_reports_unavailable(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "none", "detail": "no ros"})
    r = _NODE_REGISTRY["ROS2SystemCheck"]({"refresh": True})
    assert r["available"] is False
    assert r["backend"] == "none"


def test_detect_backend_launches_docker_desktop_when_daemon_is_down(monkeypatch):
    rt._cached_backend = None
    monkeypatch.setattr(rt.shutil, "which", lambda name: None if name == "ros2" else "/usr/bin/docker")
    ready_calls = iter([False, False, True])
    monkeypatch.setattr(rt, "_docker_ok", lambda: next(ready_calls, True))
    monkeypatch.setattr(rt, "_docker_desktop_executable", lambda: Path("Docker Desktop.exe"))
    launched = []
    monkeypatch.setattr(rt.subprocess, "Popen", lambda *a, **k: launched.append(a))
    monkeypatch.setattr(rt.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(rt.sys, "platform", "win32")

    result = rt.detect_backend(refresh=True)

    assert launched, "Docker Desktop should have been launched"
    assert result["backend"] == "docker"


def test_detect_backend_reports_docker_launch_failure(monkeypatch):
    rt._cached_backend = None
    monkeypatch.setattr(rt.shutil, "which", lambda name: None if name == "ros2" else "/usr/bin/docker")
    monkeypatch.setattr(rt, "_docker_ok", lambda: False)
    monkeypatch.setattr(rt, "_docker_desktop_executable", lambda: None)

    result = rt.detect_backend(refresh=True)

    assert result["backend"] == "none"
    assert "Docker" in result["detail"]


def test_echo_keeps_partial_messages_on_timeout(monkeypatch):
    fake = {
        "ok": False, "timed_out": True, "backend": "docker",
        "stdout": "data: a\n---\ndata: b\n---", "stderr": "", "error": "timed out",
    }
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: fake)
    r = _NODE_REGISTRY["ROS2TopicEcho"]({"topic": "/chatter", "count": 5})
    assert len(r["messages"]) == 2
    assert "received 2" in r["report"]


def test_topic_publisher_builds_managed_continuous_publish_command(monkeypatch):
    captured = {}

    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(rt, "stop_ros2_managed", lambda key, pattern="": {"ok": True, "backend": "native", "stopped": 0})

    def fake_managed(key, args):
        captured["key"] = key
        captured["args"] = args
        return {"ok": True, "backend": "native"}

    monkeypatch.setattr(rt, "run_ros2_managed", fake_managed)
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: {
        "ok": True,
        "backend": "native",
        "stdout": "/events",
        "stderr": "",
    })

    result = _NODE_REGISTRY["ROS2TopicPublisher"]({
        "action": "start",
        "topic": "/events",
        "msg_type": "std_msgs/msg/String",
        "payload": "data: reusable",
        "rate_hz": 4.0,
    })

    assert captured == {
        "key": "topic-publisher:/events",
        "args": [
            "topic", "pub", "-r", "4.0", "/events",
            "std_msgs/msg/String", "data: reusable",
        ],
    }
    assert result["running"] is True
    assert result["backend"] == "native"
    assert "topic publisher running" in result["report"]


def test_topic_publisher_start_replaces_older_docker_publishers_on_same_topic(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "test"})

    def fake_stop(key, pattern=""):
        captured.setdefault("stops", []).append((key, pattern))
        return {"ok": True, "backend": "docker", "stopped": 3}

    monkeypatch.setattr(rt, "stop_ros2_managed", fake_stop)
    monkeypatch.setattr(
        rt,
        "run_ros2_managed",
        lambda key, args: {"ok": True, "backend": "docker"},
    )
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: {
        "ok": True,
        "backend": "docker",
        "stdout": "/joint_states",
        "stderr": "",
    })

    result = _NODE_REGISTRY["ROS2TopicPublisher"]({
        "action": "start",
        "topic": "/joint_states",
        "msg_type": "sensor_msgs/msg/JointState",
        "payload": "{name: ['joint'], position: [0.0]}",
        "rate_hz": 10.0,
    })

    assert captured["stops"] == [
        ("topic-publisher:/joint_states", "ros2 topic pub .* /joint_states "),
    ]
    assert result["running"] is True


def test_topic_publisher_stop_is_scoped_to_topic(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "test"})

    def fake_stop(key, pattern=""):
        captured["key"] = key
        captured["pattern"] = pattern
        return {"ok": True, "backend": "docker", "stopped": 1}

    monkeypatch.setattr(rt, "stop_ros2_managed", fake_stop)

    result = _NODE_REGISTRY["ROS2TopicPublisher"]({
        "action": "stop",
        "topic": "/events",
    })

    assert captured == {
        "key": "topic-publisher:/events",
        "pattern": "ros2 topic pub .* /events ",
    }
    assert result["running"] is False
    assert result["backend"] == "docker"


def test_topic_publisher_rejects_invalid_rate_without_starting(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(
        rt,
        "run_ros2_managed",
        lambda *args, **kwargs: pytest.fail("invalid configuration must not start a publisher"),
    )

    result = _NODE_REGISTRY["ROS2TopicPublisher"]({"rate_hz": 0})

    assert result["running"] is False
    assert "rate_hz must be greater than 0" in result["report"]


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


def test_run_ros2_managed_docker_reports_missing_package_when_install_fails(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "x"})
    monkeypatch.setattr(rt, "ensure_container", lambda: None)
    monkeypatch.setattr(rt, "stop_ros2_managed", lambda key, pattern="": {"ok": True, "stopped": 0})
    calls = []

    def fake_run(cmd, timeout):
        calls.append(cmd)
        if "pkg prefix" in cmd[-1]:
            return SimpleNamespace(returncode=1, stdout="", stderr="Package 'image_tools' not found")
        if "apt-get install" in cmd[-1]:
            return SimpleNamespace(returncode=1, stdout="", stderr="Unable to locate package ros-jazzy-image-tools")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(rt, "_run", fake_run)

    result = rt.run_ros2_managed("camera_driver", ["run", "image_tools", "cam2image"])

    assert result["ok"] is False
    assert "image_tools" in result["error"]
    assert "installing it automatically failed" in result["error"]
    assert not any(cmd[:3] == ["docker", "exec", "-d"] for cmd in calls)


def test_run_ros2_managed_docker_installs_missing_package_then_starts(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "x"})
    monkeypatch.setattr(rt, "ensure_container", lambda: None)
    monkeypatch.setattr(rt, "stop_ros2_managed", lambda key, pattern="": {"ok": True, "stopped": 0})
    calls = []
    prefix_checks = {"count": 0}

    def fake_run(cmd, timeout):
        calls.append(cmd)
        if "pkg prefix" in cmd[-1]:
            prefix_checks["count"] += 1
            # missing on the first check, installed by the time of the recheck
            ok = prefix_checks["count"] > 1
            return SimpleNamespace(returncode=0 if ok else 1, stdout="", stderr="")
        if "apt-get install" in cmd[-1]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(rt, "_run", fake_run)

    result = rt.run_ros2_managed("camera_driver", ["run", "image_tools", "cam2image"])

    assert result["ok"] is True
    assert any("apt-get install" in cmd[-1] for cmd in calls)
    assert any(cmd[:3] == ["docker", "exec", "-d"] for cmd in calls)


def test_run_ros2_managed_docker_starts_after_package_check_passes(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "x"})
    monkeypatch.setattr(rt, "ensure_container", lambda: None)
    monkeypatch.setattr(rt, "stop_ros2_managed", lambda key, pattern="": {"ok": True, "stopped": 0})
    calls = []

    def fake_run(cmd, timeout):
        calls.append(cmd)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(rt, "_run", fake_run)

    result = rt.run_ros2_managed("camera_driver", ["run", "image_tools", "cam2image"])

    assert result["ok"] is True
    assert any(cmd[:3] == ["docker", "exec", "-d"] for cmd in calls)


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


def test_host_camera_url_is_rewritten_so_the_container_can_reach_the_host():
    # 127.0.0.1 inside the container is the container itself, so a host stream
    # on loopback is invisible until it is rewritten.
    assert rt.container_reachable_url("http://127.0.0.1:39000/stream.mjpg") == (
        "http://host.docker.internal:39000/stream.mjpg"
    )
    assert rt.container_reachable_url("http://localhost:8080/s") == "http://host.docker.internal:8080/s"
    assert rt.container_reachable_url("http://192.168.1.5:8080/s") == "http://192.168.1.5:8080/s"


def test_docker_stream_waits_for_real_http_not_just_an_open_port(monkeypatch):
    # Docker publishes ports through a proxy that accepts TCP before the
    # server inside the container is serving. Reporting ready on TCP alone
    # left the editor's <img> pointed at a dead port, which it never retries.
    class FakeProc:
        def poll(self):
            return None

    monkeypatch.setattr(rt, "ensure_container", lambda: None)
    monkeypatch.setattr(rt, "_ensure_container_stream_deps", lambda: None)
    monkeypatch.setattr(rt, "_copy_to_container", lambda *a, **k: None)
    monkeypatch.setattr(rt, "stop_image_stream", lambda stream_id="": {"ok": True, "stopped": 0})
    monkeypatch.setattr(rt, "_free_docker_stream_port", lambda preferred=0: (39000, ""))
    monkeypatch.setattr(rt.subprocess, "Popen", lambda *a, **k: FakeProc())
    monkeypatch.setattr(rt.time, "sleep", lambda seconds: None)
    # a bare TCP connect always succeeds here, exactly like the docker proxy
    monkeypatch.setattr(rt, "_port_open", lambda host, port, timeout=0.15: True)
    http_calls = {"count": 0}

    def fake_http_ready(host, port, timeout=0.6):
        http_calls["count"] += 1
        return http_calls["count"] > 3  # not serving yet on the first probes

    monkeypatch.setattr(rt, "_stream_http_ready", fake_http_ready)
    rt._streams.clear()

    result = rt._start_docker_image_stream(
        stream_id="camera", topic="/camera/image_raw", message_type="raw",
        host="127.0.0.1", port=0, max_fps=10.0, max_width=960, jpeg_quality=80,
    )

    assert result["ok"] is True
    assert http_calls["count"] > 3, "must keep probing HTTP until the server really answers"
    assert result["stream_url"] == "http://127.0.0.1:39000/stream.mjpg"
    rt._streams.clear()


def test_docker_stream_port_allocator_uses_runtime_state(monkeypatch):
    class FakeProc:
        def poll(self):
            return None

    monkeypatch.setattr(rt, "STREAM_PORT_RANGE", "39000-39002")
    rt._streams.clear()
    rt._streams["a"] = {"backend": "docker", "port": 39000, "proc": FakeProc()}

    assert rt._free_docker_stream_port() == (39001, "")
    assert rt._free_docker_stream_port(39002) == (39002, "")
    assert rt._free_docker_stream_port(38999)[1].startswith("Docker CameraROS2Subscribe port must be within")


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
        "leader_followers": 0,
        "policy_runs": 0,
    }
    assert rt._streams == {}
    assert rt._managed_detached == {}
    assert rt._detached == []


# --- rosbridge transport primitives -----------------------------------------------

def test_rosbridge_string_control_publish_waits_and_repeats(monkeypatch):
    events = []

    class FakeTopic:
        def __init__(self, ros, topic, message_type):
            events.append(("topic", topic, message_type))

        def advertise(self):
            events.append(("advertise",))

        def publish(self, message):
            events.append(("publish", message))

        def unadvertise(self):
            events.append(("unadvertise",))

    monkeypatch.setattr(rb, "get_connection", lambda *a, **k: object())
    monkeypatch.setattr(rb, "roslibpy", SimpleNamespace(Topic=FakeTopic, Message=lambda value: value))
    monkeypatch.setattr(rb.time, "sleep", lambda seconds: events.append(("sleep", seconds)))

    result = rb.publish_string("127.0.0.1", 9090, "/robot_control", '{"action":"enter_teach"}')

    assert result == {"ok": True, "sent": 3}
    assert len([event for event in events if event[0] == "publish"]) == 3
    assert events[-1] == ("unadvertise",)


def test_joint_stream_seed_config_replaces_stale_torque_state():
    session = rb.JointStreamSession.__new__(rb.JointStreamSession)
    session._data_lock = threading.Lock()
    session._config_event = threading.Event()
    session._config = {"torque_enabled": True, "mode": "hold"}

    session.seed_config({"torque_enabled": False, "mode": "teach"})

    assert session.wait_for_config(0) == {"torque_enabled": False, "mode": "teach"}


def test_joint_stream_release_retains_idle_subscription_until_explicit_stop(monkeypatch):
    key = ("127.0.0.1", 9090, "/joint_states", "/joint_commands", "/joint_config")
    closed = []
    session = SimpleNamespace(key=key, _users=1, close=lambda: closed.append(True))
    monkeypatch.setattr(rb, "_joint_streams", {key: session})

    rb.release_joint_stream(session)

    assert session._users == 0
    assert rb._joint_streams[key] is session
    assert closed == []

    assert rb.close_joint_streams() == 1
    assert rb._joint_streams == {}
    assert closed == [True]


def test_joint_stream_release_can_discard_a_stale_subscription(monkeypatch):
    key = ("127.0.0.1", 9090, "/joint_states", "/joint_commands", "/joint_config")
    closed = []
    session = SimpleNamespace(key=key, _users=1, close=lambda: closed.append(True))
    monkeypatch.setattr(rb, "_joint_streams", {key: session})

    rb.release_joint_stream(session, discard=True)

    assert session._users == 0
    assert rb._joint_streams == {}
    assert closed == [True]


def test_joint_stream_discard_replaces_stale_shared_subscription(monkeypatch):
    key = ("127.0.0.1", 9090, "/joint_states", "/joint_commands", "/joint_config")
    closed = []
    session = SimpleNamespace(key=key, _users=2, close=lambda: closed.append(True))
    monkeypatch.setattr(rb, "_joint_streams", {key: session})

    rb.release_joint_stream(session, discard=True)

    assert session._users == 0
    assert rb._joint_streams == {}
    assert closed == [True]


# --- transport preflight diagnostics ----------------------------------------------

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


def test_live_nodes_structured_error_without_roslibpy(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (False, "roslibpy is not installed"))
    status = _NODE_REGISTRY["ROS2RosbridgeStatus"]({})
    assert status["ready"] is False
    assert "MISSING" in status["report"]


# --- integration (needs native ros2 or Docker) ------------------------------------

@backend_only
def test_system_check_live():
    r = _NODE_REGISTRY["ROS2SystemCheck"]({"refresh": True})
    assert r["available"] is True, r["report"]
    assert r["backend"] in ("native", "docker")


@backend_only
def test_publish_then_echo_roundtrip():
    start = _NODE_REGISTRY["ROS2TopicPublisher"](
        {
            "action": "start",
            "topic": "/bn_test",
            "payload": "data: roundtrip",
            "rate_hz": 5.0,
        }
    )
    assert start["running"] is True, start["report"]
    try:
        r = _NODE_REGISTRY["ROS2TopicEcho"]({"topic": "/bn_test", "count": 1, "timeout": 30.0})
        assert r["messages"], r["report"]
        assert "roundtrip" in r["messages"][0]

        topics = _NODE_REGISTRY["ROS2TopicList"]({"show_types": False})
        assert any("/bn_test" in t for t in topics["topics"]), topics
    finally:
        _NODE_REGISTRY["ROS2TopicPublisher"]({"action": "stop", "topic": "/bn_test"})


@backend_only
def test_interface_show_live():
    r = _NODE_REGISTRY["ROS2InterfaceShow"]({"interface": "std_msgs/msg/String"})
    assert "string data" in r["definition"], r["report"]
