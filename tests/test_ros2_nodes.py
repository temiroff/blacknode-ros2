"""blacknode-ros2 — integration primitive contracts.

Graph discovery, topics, services, processes, and the native/rosbridge
transports. Capability nodes built on these (joint control, camera streaming)
live in their own packages' ROS 2 adapters and are tested there.

The no-backend contract (structured error, never raises) is always exercised.
Integration tests run only when a real backend (native ros2 or Docker) is
available, and skip cleanly otherwise.
"""
import io
import json
import shutil
import subprocess
import sys
import threading
import time
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import (
    _PACKAGE_REGISTRY,
    component_dependency_plan,
    load_package,
)

PACKAGE_DIR = Path(__file__).resolve().parents[1]
with patch(
    "blacknode.packages._read_component_overrides",
    return_value=({
        "core": True,
        "topics": True,
        "services": True,
        "diagnostics": True,
        "rosbridge": True,
        "processes": True,
    }, ""),
):
    load_package(PACKAGE_DIR)

from blacknode.pkg.blacknode_ros2 import ros2_runtime as rt
from blacknode.pkg.blacknode_ros2 import ros2_live as live
from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
from blacknode.pkg.blacknode_ros2 import rosbridge_service as service
from blacknode.workflow import validate_workflow

TEMPLATE_DIR = PACKAGE_DIR / "templates"

EXPECTED_NODES = [
    "ROS2",
    "ROS2BridgeEcho",
    "ROS2BridgePublish",
    "ROS2GraphExplorer",
    "ROS2InterfaceShow",
    "ROS2Launch",
    "ROS2NodeList",
    "ROS2PackageExecutables",
    "ROS2PythonNode",
    "ROS2RosbridgeServer",
    "ROS2RosbridgeStatus",
    "ROS2Run",
    "ROS2ServiceList",
    "ROS2Status",
    "ROS2SystemCheck",
    "ROS2TopicEcho",
    "ROS2TopicList",
    "ROS2TopicPublisher",
    "ROS2TopicRelay",
    "ROS2TopicSubscriber",
    "ROS2VisualDashboard",
    "ROS2WorkspaceBuild",
]


def test_native_components_are_default_and_remote_process_tools_are_optional():
    info = _PACKAGE_REGISTRY["blacknode-ros2"]
    assert all(info.components[name]["default"] for name in {"core", "topics", "services", "diagnostics"})
    assert info.components["rosbridge"]["default"] is False
    assert info.components["processes"]["default"] is False

EXPECTED_COMPONENT_NODES = {
    "core": set(),
    "rosbridge": {
        "ROS2BridgeEcho",
        "ROS2BridgePublish",
        "ROS2RosbridgeServer",
        "ROS2RosbridgeStatus",
    },
    "topics": {
        "ROS2",
        "ROS2TopicEcho",
        "ROS2TopicList",
        "ROS2TopicPublisher",
        "ROS2TopicRelay",
        "ROS2TopicSubscriber",
    },
    "services": {"ROS2ServiceList"},
    "processes": {
        "ROS2Launch", "ROS2PackageExecutables", "ROS2PythonNode", "ROS2Run",
        "ROS2WorkspaceBuild",
    },
    "diagnostics": {
        "ROS2GraphExplorer",
        "ROS2InterfaceShow",
        "ROS2NodeList",
        "ROS2Status",
        "ROS2SystemCheck",
        "ROS2VisualDashboard",
    },
}

HAS_BACKEND = rt._passive_backend() != "none"
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

    expected_nodes = sorted(
        EXPECTED_COMPONENT_NODES["services"]
        | EXPECTED_COMPONENT_NODES["diagnostics"]
    )
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

    assert "ROS2TopicPublish" not in _NODE_REGISTRY
    assert "ROS2DemoPublisher" not in _NODE_REGISTRY
    assert publisher._bn_inputs == [
        "trigger", "action", "node_name", "topic", "msg_type", "payload", "count", "rate_hz",
    ]
    assert publisher._bn_input_choices["action"] == ["once", "start", "stop"]
    assert publisher._bn_outputs == ["running", "backend", "report"]
    assert publisher._bn_hidden is False
    assert _NODE_REGISTRY["ROS2VisualDashboard"]._bn_hidden is True


def test_topic_subscriber_has_managed_named_contract():
    subscriber = _NODE_REGISTRY["ROS2TopicSubscriber"]

    assert subscriber._bn_inputs == [
        "trigger", "action", "node_name", "topic", "msg_type", "history", "timeout",
    ]
    assert subscriber._bn_input_choices["action"] == ["once", "start", "stop"]
    assert subscriber._bn_outputs == [
        "running", "latest", "messages", "received", "backend", "report",
    ]
    assert subscriber._bn_hidden is False


def test_ros2_has_generic_managed_stream_contract():
    ros2 = _NODE_REGISTRY["ROS2"]

    assert ros2._bn_inputs == [
        "trigger", "device", "action", "topic", "message_type", "node_name", "history",
        "timeout", "stale_after_seconds",
    ]
    assert ros2._bn_input_choices["action"] == ["once", "start", "status", "stop"]
    assert ros2._bn_outputs == [
        "running", "message", "messages", "stream", "status", "received",
        "backend", "report",
    ]
    assert ros2._bn_primary_inputs == ["device", "action", "topic", "message_type"]
    assert ros2._bn_primary_outputs == ["stream", "status", "message"]
    assert ros2._bn_live_capable is True


def test_python_node_has_standalone_script_contract():
    python_node = _NODE_REGISTRY["ROS2PythonNode"]

    assert python_node._bn_inputs == [
        "trigger", "action", "run_id", "source_mode", "script_path", "code", "arguments",
    ]
    assert python_node._bn_input_choices["action"] == ["start", "stop"]
    assert python_node._bn_input_choices["source_mode"] == ["file", "inline"]
    assert python_node._bn_outputs == [
        "running", "run_id", "backend", "script", "logs", "report",
    ]
    assert python_node._bn_primary_inputs == [
        "trigger", "action", "run_id", "source_mode", "script_path",
    ]
    assert python_node._bn_primary_outputs == ["running", "logs", "report"]
    assert python_node._bn_hidden is False


def test_workspace_build_has_colcon_contract():
    build = _NODE_REGISTRY["ROS2WorkspaceBuild"]

    assert build._bn_inputs == [
        "trigger", "workspace_path", "packages_select", "timeout",
    ]
    assert build._bn_outputs == [
        "built", "backend", "workspace_path", "setup_path", "logs", "report",
    ]
    assert build._bn_primary_inputs == [
        "trigger", "workspace_path", "packages_select",
    ]
    assert build._bn_primary_outputs == ["built", "logs", "report"]
    assert build._bn_hidden is False


def test_topic_relay_has_generic_data_contract():
    relay = _NODE_REGISTRY["ROS2TopicRelay"]

    assert relay._bn_inputs == [
        "trigger", "action", "run_id", "source_topic", "destination_topic",
        "msg_type", "qos", "queue_depth",
    ]
    assert relay._bn_outputs == [
        "running", "backend", "source_topic", "destination_topic", "report",
    ]
    assert relay._bn_hidden is False


def test_graph_explorer_has_read_only_topology_contract():
    explorer = _NODE_REGISTRY["ROS2GraphExplorer"]

    assert explorer._bn_inputs == [
        "trigger", "namespace", "include_system", "include_endpoints",
        "max_topics", "timeout",
    ]
    assert explorer._bn_outputs == [
        "available", "backend", "graph", "nodes", "topics", "services",
        "report",
    ]
    assert explorer._bn_hidden is False


def test_capability_nodes_are_not_owned_by_the_integration_layer():
    """Camera and arm-control nodes belong to their capability packages.

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
    monkeypatch.setattr(service, "_start_native_rosbridge", lambda *args, **kwargs: None)
    monkeypatch.setattr(service, "_start_docker_desktop", lambda timeout: (_ for _ in ()).throw(RuntimeError("install Docker Desktop")))

    result = _NODE_REGISTRY["ROS2RosbridgeServer"]({"action": "ensure"})

    assert result["ready"] is False
    assert "install Docker Desktop" in result["report"]


def test_rosbridge_server_reports_linux_docker_setup_command(monkeypatch):
    monkeypatch.setattr(service, "_docker_ready", lambda: False)
    monkeypatch.setattr(service.platform, "system", lambda: "Linux")
    monkeypatch.setattr(service.shutil, "which", lambda command: None)

    with pytest.raises(RuntimeError, match=r"\./service\.sh docker"):
        service._start_docker_desktop(30)


def test_rosbridge_server_reports_linux_docker_daemon_problem(monkeypatch):
    monkeypatch.setattr(service, "_docker_ready", lambda: False)
    monkeypatch.setattr(service.platform, "system", lambda: "Linux")
    monkeypatch.setattr(service.shutil, "which", lambda command: "/usr/bin/docker")

    with pytest.raises(RuntimeError, match="daemon or socket"):
        service._start_docker_desktop(30)


def test_rosbridge_image_build_allows_slow_arm_device(monkeypatch):
    port_checks = iter([False, True])
    monkeypatch.setattr(service, "_port_open", lambda host, port: next(port_checks))
    monkeypatch.setattr(service, "_start_native_rosbridge", lambda *args, **kwargs: None)
    monkeypatch.setattr(service, "_start_docker_desktop", lambda timeout: None)
    calls = []

    def fake_run(command, timeout, **kwargs):
        calls.append((command, timeout, kwargs))
        return subprocess.CompletedProcess(
            command,
            1 if command[:3] in [
                ["docker", "image", "inspect"],
                ["docker", "container", "inspect"],
            ] else 0,
            stdout="container-id",
            stderr="",
        )

    monkeypatch.setattr(service, "_run", fake_run)

    report = service.ensure_local_rosbridge("127.0.0.1", 9091, 30)

    build = next(call for call in calls if call[0][:2] == ["docker", "build"])
    assert build[1] >= 600
    assert "rosbridge ready" in report


def test_rosbridge_prefers_native_ros_on_linux(monkeypatch):
    port_checks = iter([False, True])
    commands = []

    class FakeProcess:
        pid = 4321

        def poll(self):
            return None

        def terminate(self):
            return None

        def wait(self, timeout=None):
            return 0

    monkeypatch.setattr(service, "_port_open", lambda host, port: next(port_checks))
    monkeypatch.setattr(service, "_native_ros_command", lambda args: ["/opt/ros/jazzy/bin/ros2", *args])
    monkeypatch.setattr(
        service,
        "_run",
        lambda command, timeout, **kwargs: subprocess.CompletedProcess(command, 0, stdout="/opt/ros/jazzy", stderr=""),
    )
    monkeypatch.setattr(
        service.subprocess,
        "Popen",
        lambda command, **kwargs: commands.append(command) or FakeProcess(),
    )
    monkeypatch.setattr(
        service,
        "_start_docker_desktop",
        lambda timeout: pytest.fail("native ROS 2 must be preferred"),
    )

    report = service.ensure_local_rosbridge("127.0.0.1", 9091, 30)
    service._NATIVE_PROCESSES.pop(9091, None)

    assert "native ROS 2 process 4321" in report
    assert commands[0][-2:] == ["address:=127.0.0.1", "port:=9091"]


def test_native_ros_reports_missing_rosbridge_package(monkeypatch):
    monkeypatch.setattr(service, "_native_ros_command", lambda args: ["/opt/ros/jazzy/bin/ros2", *args])
    monkeypatch.setattr(service, "_native_ros_setup", lambda: Path("/opt/ros/jazzy/setup.bash"))
    monkeypatch.setattr(
        service,
        "_run",
        lambda command, timeout, **kwargs: subprocess.CompletedProcess(command, 1, stdout="", stderr="missing"),
    )

    with pytest.raises(RuntimeError, match="ros-jazzy-rosbridge-server"):
        service._start_native_rosbridge("127.0.0.1", 9091, 30, expose_lan=False)


def test_native_joint_stream_tracks_pose_config_and_publishes(monkeypatch):
    published = []

    class FakeJointState:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)
            self.name = []
            self.position = []
            self.velocity = []
            self.effort = []

    class FakePublisher:
        def publish(self, message):
            published.append(message)

    class FakeNode:
        def create_subscription(self, *args):
            return object()

        def create_publisher(self, *args):
            return FakePublisher()

        def destroy_subscription(self, entity):
            return None

        def destroy_publisher(self, entity):
            return None

        def destroy_node(self):
            return None

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: "stamp"),
            )

    fake_node = FakeNode()
    class FakeExecutor:
        def add_node(self, node):
            return None

        def spin_once(self, timeout_sec=0):
            time.sleep(min(timeout_sec, 0.001))

        def remove_node(self, node):
            return None

        def shutdown(self, timeout_sec=0):
            return None

    fake_rclpy = SimpleNamespace(
        create_node=lambda name: fake_node,
    )
    policy = SimpleNamespace(RELIABLE="reliable", TRANSIENT_LOCAL="transient")
    monkeypatch.setattr(
        nr,
        "_ensure_rclpy",
        lambda: {
            "rclpy": fake_rclpy,
            "SingleThreadedExecutor": FakeExecutor,
            "JointState": FakeJointState,
            "String": SimpleNamespace,
            "QoSProfile": lambda **kwargs: kwargs,
            "ReliabilityPolicy": policy,
            "DurabilityPolicy": policy,
        },
    )

    session = nr.acquire_joint_stream(
        "/leader/joint_states",
        "/leader/joint_commands",
        "/leader/joint_config",
    )
    try:
        session._on_state(SimpleNamespace(name=["joint_1"], position=[0.25]))
        session._on_config(SimpleNamespace(data='{"torque_enabled": false}'))
        pose, config, age = session.snapshot()
        session.publish({"joint_1": 0.5})
    finally:
        nr.release_joint_stream(session)

    assert pose == {"joint_1": 0.25}
    assert config == {"torque_enabled": False}
    assert age < 0.1
    assert published[0].name == ["joint_1"]
    assert published[0].position == [0.5]


def test_native_read_only_joint_stream_has_semantic_name_and_no_command_publisher(monkeypatch):
    created = []
    publishers = []

    class FakeNode:
        def create_subscription(self, *args):
            return object()

        def create_publisher(self, *args):
            publishers.append(args)
            return object()

        def destroy_subscription(self, entity):
            return None

        def destroy_publisher(self, entity):
            return None

        def destroy_node(self):
            return None

    class FakeExecutor:
        def add_node(self, node):
            return None

        def spin_once(self, timeout_sec=0):
            time.sleep(min(timeout_sec, 0.001))

        def remove_node(self, node):
            return None

        def shutdown(self, timeout_sec=0):
            return None

    def create_node(name, **kwargs):
        created.append((name, kwargs))
        return FakeNode()

    policy = SimpleNamespace(RELIABLE="reliable", TRANSIENT_LOCAL="transient")
    monkeypatch.setattr(
        nr,
        "_ensure_rclpy",
        lambda: {
            "rclpy": SimpleNamespace(create_node=create_node),
            "SingleThreadedExecutor": FakeExecutor,
            "JointState": SimpleNamespace,
            "String": SimpleNamespace,
            "QoSProfile": lambda **kwargs: kwargs,
            "ReliabilityPolicy": policy,
            "DurabilityPolicy": policy,
        },
    )

    session = nr.acquire_joint_stream(
        "/leader/joint_states",
        "",
        "/leader/joint_config",
        node_name="blacknode/leader monitor",
    )
    try:
        session.seed_config({"torque_enabled": False})
        assert session.snapshot()[1] == {"torque_enabled": False}
        with pytest.raises(RuntimeError, match="read-only"):
            session.publish({"joint_1": 0.5})
    finally:
        nr.release_joint_stream(session)

    assert created == [(
        "blacknode_leader_monitor",
        {
            "enable_rosout": False,
            "start_parameter_services": False,
            "enable_type_description_service": False,
        },
    )]
    assert publishers == []


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


def test_native_string_subscription_delivers_and_closes(monkeypatch):
    received = []
    destroyed = []

    class FakeExecutor:
        def add_node(self, node):
            return None

        def spin_once(self, timeout_sec=0):
            time.sleep(min(timeout_sec, 0.001))

        def remove_node(self, node):
            return None

        def shutdown(self, timeout_sec=0):
            return None

    class FakeNode:
        def create_subscription(self, _message_type, _topic, callback, _qos):
            self.callback = callback
            return object()

        def destroy_subscription(self, entity):
            destroyed.append(entity)

        def destroy_node(self):
            return None

    fake_node = FakeNode()
    monkeypatch.setattr(
        nr,
        "_ensure_rclpy",
        lambda: {
            "rclpy": SimpleNamespace(create_node=lambda _name: fake_node),
            "SingleThreadedExecutor": FakeExecutor,
            "String": SimpleNamespace,
        },
    )

    session = nr.acquire_string_subscription("/control", received.append)
    fake_node.callback(SimpleNamespace(data='{"armed": true}'))
    nr.release_string_subscription(session)

    assert received == ['{"armed": true}']
    assert len(destroyed) == 1


def test_generic_status_falls_back_to_rosbridge_and_ensures_service(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "missing rclpy"))
    server_requests = []
    monkeypatch.setattr(
        service,
        "ros2_rosbridge_server",
        lambda ctx: server_requests.append(ctx) or {"ready": True, "report": "server ready"},
    )
    monkeypatch.setattr(live, "ros2_rosbridge_status", lambda ctx: {
        "connected": True, "ready": True, "config": {}, "report": "bridge ready",
    })

    result = _NODE_REGISTRY["ROS2Status"]({
        "transport": "auto",
        "ensure_rosbridge": True,
        "expose_lan": True,
    })

    assert result["transport"] == "rosbridge"
    assert result["ready"] is True
    assert "bridge ready" in result["report"]
    assert server_requests[0]["expose_lan"] is True


def test_rosbridge_uses_a_distinct_container_for_the_leader_port():
    assert service._container_name(9090) == "blacknode-rosbridge"
    assert service._container_name(9091) == "blacknode-rosbridge-9091"


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
        "ros2-graph-explorer.json": {
            "blacknode-ros2/core",
            "blacknode-ros2/diagnostics",
        },
            "ros2-run-your-package.json": {
                "blacknode-ros2/core",
                "blacknode-ros2/topics",
                "blacknode-ros2/processes",
                "blacknode-ros2/diagnostics",
            },
            "ros2-topic-relay.json": {
                "blacknode-ros2/core",
                "blacknode-ros2/topics",
            },
            "ros2-topic-stream.json": {
                "blacknode-ros2/core",
                "blacknode-ros2/topics",
            },
            "ros2-device-topic-stream.json": {
                "blacknode-robot/capabilities",
                "blacknode-ros2/core",
                "blacknode-ros2/topics",
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


def test_graph_explorer_maps_publishers_topics_subscribers_and_qos(monkeypatch):
    monkeypatch.setattr(
        rt,
        "detect_backend",
        lambda refresh=False: {"backend": "native", "detail": "test"},
    )

    def fake_run(args, timeout=15.0):
        outputs = {
            ("node", "list"): "/camera_driver\n/viewer",
            ("topic", "list", "-t"): (
                "/camera/image_raw [sensor_msgs/msg/Image]\n"
                "/parameter_events [rcl_interfaces/msg/ParameterEvent]\n"
            ),
            ("service", "list", "-t"): "/camera/set_power [std_srvs/srv/SetBool]",
            ("topic", "info", "-v", "/camera/image_raw"): (
                "Type: sensor_msgs/msg/Image\n"
                "Publisher count: 1\n\n"
                "Node name: camera_driver\n"
                "Node namespace: /\n"
                "Topic type: sensor_msgs/msg/Image\n"
                "Endpoint type: PUBLISHER\n"
                "GID: 01.02\n"
                "QoS profile:\n"
                "  Reliability: BEST_EFFORT\n"
                "  Durability: VOLATILE\n\n"
                "Subscription count: 1\n\n"
                "Node name: viewer\n"
                "Node namespace: /\n"
                "Topic type: sensor_msgs/msg/Image\n"
                "Endpoint type: SUBSCRIPTION\n"
                "GID: 03.04\n"
                "QoS profile:\n"
                "  Reliability: BEST_EFFORT\n"
                "  Durability: VOLATILE\n"
            ),
        }
        key = tuple(args)
        assert key in outputs, key
        return {
            "ok": True,
            "backend": "native",
            "stdout": outputs[key],
            "stderr": "",
        }

    monkeypatch.setattr(rt, "run_ros2", fake_run)
    result = _NODE_REGISTRY["ROS2GraphExplorer"]({})

    assert result["available"] is True
    assert result["nodes"] == ["/camera_driver", "/viewer"]
    assert len(result["topics"]) == 1
    topic = result["topics"][0]
    assert topic["name"] == "/camera/image_raw"
    assert topic["types"] == ["sensor_msgs/msg/Image"]
    assert topic["publisher_count"] == 1
    assert topic["subscription_count"] == 1
    assert topic["publishers"][0]["node"] == "/camera_driver"
    assert topic["publishers"][0]["qos"]["reliability"] == "BEST_EFFORT"
    assert topic["subscribers"][0]["node"] == "/viewer"
    assert result["services"] == [{
        "name": "/camera/set_power",
        "types": ["std_srvs/srv/SetBool"],
    }]
    assert result["graph"]["truncated"] is False
    assert "1 topics" in result["report"]


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

    r = _NODE_REGISTRY["ROS2TopicRelay"]({
        "source_topic": "/source",
        "destination_topic": "/destination",
    })
    assert r["running"] is False
    assert r["backend"] == "none"
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2Launch"]({"package": "demo_nodes_cpp", "launch_file": "talker.launch.py"})
    assert r["launched"] is False
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2Run"]({"package": "demo_nodes_cpp", "executable": "talker"})
    assert r["running"] is False
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2PythonNode"]({
        "source_mode": "inline",
        "code": "print('hello')",
    })
    assert r["running"] is False
    assert r["backend"] == "none"
    assert "FAILED" in r["report"]

    r = _NODE_REGISTRY["ROS2GraphExplorer"]({})
    assert r["available"] is False
    assert r["graph"]["topics"] == []
    assert r["graph"]["errors"]


def test_system_check_reports_unavailable(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "none", "detail": "no ros"})
    r = _NODE_REGISTRY["ROS2SystemCheck"]({"refresh": True})
    assert r["available"] is False
    assert r["backend"] == "none"


def test_runtime_status_never_probes_or_starts_docker(monkeypatch):
    monkeypatch.setattr(rt, "_cached_backend", None)
    monkeypatch.setattr(rt.shutil, "which", lambda name: "/usr/bin/docker" if name == "docker" else None)
    monkeypatch.setattr(
        rt,
        "detect_backend",
        lambda *args, **kwargs: pytest.fail("passive runtime status must not detect or start Docker"),
    )
    monkeypatch.setattr(
        rt,
        "_docker_ok",
        lambda: pytest.fail("passive runtime status must not probe the Docker daemon"),
    )

    result = rt.runtime_status()

    assert result["ok"] is True
    assert result["backend"] == "none"


def test_empty_stream_stop_never_probes_or_starts_docker(monkeypatch):
    monkeypatch.setattr(rt, "_cached_backend", None)
    monkeypatch.setattr(rt, "_streams", {})
    monkeypatch.setattr(rt.shutil, "which", lambda name: "/usr/bin/docker" if name == "docker" else None)
    monkeypatch.setattr(
        rt,
        "detect_backend",
        lambda *args, **kwargs: pytest.fail("empty stop must not detect or start Docker"),
    )

    result = rt.stop_image_stream("")

    assert result == {"ok": True, "backend": "none", "stopped": 0}


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


def test_topic_publisher_once_builds_bounded_publish_command(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(
        rt,
        "run_ros2_managed",
        lambda *args, **kwargs: pytest.fail("one-shot publishing must not start a managed process"),
    )

    def fake_run(args, timeout=15.0):
        captured["args"] = args
        captured["timeout"] = timeout
        return {"ok": True, "backend": "native", "stdout": "published", "stderr": ""}

    monkeypatch.setattr(rt, "run_ros2", fake_run)
    result = _NODE_REGISTRY["ROS2TopicPublisher"]({
        "action": "once",
        "node_name": "event_once",
        "topic": "/events",
        "msg_type": "std_msgs/msg/String",
        "payload": "data: bounded",
        "count": 3,
        "rate_hz": 0,
    })

    assert captured == {
        "args": [
            "topic", "pub", "--times", "3", "--wait-matching-subscriptions", "0",
            "--node-name", "event_once",
            "/events", "std_msgs/msg/String", "data: bounded",
        ],
        "timeout": 33,
    }
    assert result["running"] is False
    assert result["backend"] == "native"
    assert "publish 3x to /events OK" in result["report"]


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
        "node_name": "event_talker",
        "topic": "/events",
        "msg_type": "std_msgs/msg/String",
        "payload": "data: reusable",
        "rate_hz": 4.0,
    })

    assert captured == {
        "key": "topic-publisher:/events",
        "args": [
            "topic", "pub", "-r", "4.0", "--node-name", "event_talker", "/events",
            "std_msgs/msg/String", "data: reusable",
        ],
    }
    assert result["running"] is True
    assert result["backend"] == "native"
    assert "topic publisher running" in result["report"]
    assert "/event_talker" in result["report"]


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


def test_topic_publisher_rejects_invalid_node_name_without_starting(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(
        rt,
        "run_ros2_managed",
        lambda *args, **kwargs: pytest.fail("invalid configuration must not start a publisher"),
    )

    result = _NODE_REGISTRY["ROS2TopicPublisher"]({"node_name": "not/a/node"})

    assert result["running"] is False
    assert "node_name" in result["report"]


def test_topic_subscriber_starts_named_managed_subscription(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "test"})

    def fake_start(**kwargs):
        captured.update(kwargs)
        return {"ok": True, "backend": "docker", "run_id": "topic-subscriber:/events"}

    monkeypatch.setattr(rt, "start_topic_subscriber", fake_start)
    result = _NODE_REGISTRY["ROS2TopicSubscriber"]({
        "action": "start",
        "node_name": "event_listener",
        "topic": "/events",
        "msg_type": "std_msgs/msg/String",
        "history": 7,
    })

    assert captured == {
        "topic": "/events",
        "message_type": "std_msgs/msg/String",
        "node_name": "event_listener",
        "history": 7,
    }
    assert result["running"] is True
    assert result["messages"] == []
    assert "/event_listener" in result["report"]


def test_native_topic_subscriber_uses_ros_compatible_python(monkeypatch):
    captured = {}

    class Process:
        stdout = io.StringIO("")
        stderr = io.StringIO("")

        def poll(self):
            return None

    class Thread:
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            pass

    def popen(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return Process()

    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native"})
    monkeypatch.setattr(rt, "run_ros2", lambda *args, **kwargs: {"ok": True})
    monkeypatch.setattr(rt, "stop_topic_subscriber", lambda topic: {"ok": True})
    monkeypatch.setattr(rt, "_native_ros_python", lambda: ("/usr/bin/python3", ""))
    monkeypatch.setattr(rt.subprocess, "Popen", popen)
    monkeypatch.setattr(rt.threading, "Thread", Thread)
    monkeypatch.setattr(rt.time, "sleep", lambda _seconds: None)

    result = rt.start_topic_subscriber(
        topic="/scan",
        message_type="sensor_msgs/msg/LaserScan",
        node_name="blacknode_scan",
    )

    assert result["ok"] is True
    assert captured["command"][0] == "/usr/bin/python3"
    rt._topic_subscribers.clear()
    rt._managed_detached.clear()


def test_native_ros_python_prefers_system_interpreter_with_rclpy(monkeypatch):
    calls = []
    monkeypatch.delenv("BLACKNODE_ROS2_PYTHON", raising=False)
    monkeypatch.setattr(rt.shutil, "which", lambda name: "/usr/bin/python3")

    def probe(command, timeout):
        calls.append((command, timeout))
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(rt, "_run", probe)

    interpreter, error = rt._native_ros_python()

    assert interpreter == "/usr/bin/python3"
    assert error == ""
    assert calls[0][0][0] == "/usr/bin/python3"


def test_topic_subscriber_normalizes_nonfinite_sensor_values(monkeypatch):
    rclpy = ModuleType("rclpy")
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy_qos = ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    rosidl = ModuleType("rosidl_runtime_py")
    rosidl_convert = ModuleType("rosidl_runtime_py.convert")
    rosidl_convert.message_to_ordereddict = lambda message: message
    rosidl_utilities = ModuleType("rosidl_runtime_py.utilities")
    rosidl_utilities.get_message = lambda name: name
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    monkeypatch.setitem(sys.modules, "rclpy.qos", rclpy_qos)
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py", rosidl)
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py.convert", rosidl_convert)
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py.utilities", rosidl_utilities)
    script = PACKAGE_DIR / "scripts" / "ros2_topic_subscriber.py"
    spec = spec_from_file_location("blacknode_ros2_topic_subscriber_test", script)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    result = module._json_safe({
        "ranges": [1.0, float("nan"), float("inf"), -float("inf")],
    })

    assert result == {"ranges": [1.0, None, None, None]}
    assert json.dumps(result, allow_nan=False)


def test_topic_subscriber_once_returns_structured_message(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(rt, "run_topic_subscriber_once", lambda **kwargs: {
        "ok": True,
        "running": False,
        "backend": "native",
        "messages": [{"data": "hello"}],
        "latest": {"data": "hello"},
        "received": 1,
    })

    result = _NODE_REGISTRY["ROS2TopicSubscriber"]({
        "action": "once",
        "node_name": "event_listener",
        "topic": "/events",
        "timeout": 3.0,
    })

    assert result["running"] is False
    assert result["latest"] == {"data": "hello"}
    assert result["messages"] == [{"data": "hello"}]
    assert result["received"] == 1


def test_topic_subscriber_stop_is_scoped_to_topic(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "test"})

    def fake_stop(topic):
        captured["topic"] = topic
        return {"ok": True, "backend": "docker", "messages": [{"data": "last"}], "received": 4}

    monkeypatch.setattr(rt, "stop_topic_subscriber", fake_stop)
    result = _NODE_REGISTRY["ROS2TopicSubscriber"]({"action": "stop", "topic": "/events"})

    assert captured == {"topic": "/events"}
    assert result["running"] is False
    assert result["latest"] == {"data": "last"}
    assert result["received"] == 4


def test_topic_subscriber_rejects_invalid_node_name(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(
        rt,
        "start_topic_subscriber",
        lambda **kwargs: pytest.fail("invalid configuration must not start a subscriber"),
    )

    result = _NODE_REGISTRY["ROS2TopicSubscriber"]({"node_name": "not/a/node"})

    assert result["running"] is False
    assert "node_name" in result["report"]


def test_generic_ros2_starts_managed_topic_stream(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})

    def fake_start(**kwargs):
        captured.update(kwargs)
        return {"ok": True, "backend": "native", "run_id": "topic-subscriber:/scan"}

    monkeypatch.setattr(rt, "start_topic_subscriber", fake_start)
    monkeypatch.setattr(rt, "topic_subscriber_status", lambda topic: {
        "ok": True,
        "running": True,
        "backend": "native",
        "topic": topic,
        "message_type": "sensor_msgs/msg/LaserScan",
        "service_id": f"topic-subscriber:{topic}",
        "messages": [],
        "latest": {},
        "received": 0,
        "last_message_time_ns": 0,
        "age_seconds": None,
        "stale_after_seconds": 1.0,
        "source_fresh": False,
        "error": "",
    })

    result = _NODE_REGISTRY["ROS2"]({
        "action": "start",
        "topic": "/scan",
        "message_type": "sensor_msgs/msg/LaserScan",
        "node_name": "blacknode_scan",
        "history": 5,
        "stale_after_seconds": 1.0,
    })

    assert captured == {
        "topic": "/scan",
        "message_type": "sensor_msgs/msg/LaserScan",
        "node_name": "blacknode_scan",
        "history": 5,
        "public_node_type": "ROS2",
        "stale_after_seconds": 1.0,
    }
    assert result["running"] is True
    assert result["stream"]["kind"] == "blacknode.message-stream"
    assert result["stream"]["protocol"] == "ros2"
    assert result["status"]["state"] == "waiting"


def test_generic_ros2_routes_connected_compute_device_to_editor_runtime(monkeypatch):
    captured = {}

    def remote_action(request):
        captured.update(request)
        return {
            "outputs": {
                "running": True,
                "message": {"ranges": [1.0, 2.0]},
                "messages": [{"ranges": [1.0, 2.0]}],
                "stream": {"kind": "blacknode.message-stream", "topic": "/scan"},
                "status": {"kind": "blacknode.stream-status", "state": "ready"},
                "received": 4,
                "backend": "remote:jetson",
                "report": "ROS2 streaming from jetson",
            },
        }

    result = _NODE_REGISTRY["ROS2"]({
        "__node_id__": "scan-node",
        "__remote_ros2_action__": remote_action,
        "device": {
            "kind": "blacknode.compute-device-target",
            "device_id": "jetson",
        },
        "action": "start",
        "topic": "/scan",
        "message_type": "sensor_msgs/msg/LaserScan",
    })

    assert captured["node_id"] == "scan-node"
    assert captured["device_id"] == "jetson"
    assert captured["action"] == "start"
    assert captured["topic"] == "/scan"
    assert result["received"] == 4
    assert result["message"]["ranges"] == [1.0, 2.0]


def test_generic_ros2_remote_target_is_structurally_unavailable_outside_editor():
    result = _NODE_REGISTRY["ROS2"]({
        "device": {"device_id": "jetson"},
        "action": "status",
        "topic": "/scan",
    })

    assert result["running"] is False
    assert result["status"]["state"] == "unavailable"
    assert "editor Runtime" in result["report"]


def test_generic_ros2_status_reports_fresh_message(monkeypatch):
    monkeypatch.setattr(rt, "topic_subscriber_status", lambda topic: {
        "ok": True,
        "running": True,
        "backend": "native",
        "topic": topic,
        "message_type": "std_msgs/msg/String",
        "service_id": f"topic-subscriber:{topic}",
        "messages": [{"data": "ready"}],
        "latest": {"data": "ready"},
        "received": 3,
        "last_message_time_ns": 42,
        "age_seconds": 0.1,
        "stale_after_seconds": 2.0,
        "source_fresh": True,
        "error": "",
    })

    result = _NODE_REGISTRY["ROS2"]({"action": "status", "topic": "/events"})

    assert result["message"] == {"data": "ready"}
    assert result["received"] == 3
    assert result["status"]["state"] == "ready"
    assert result["status"]["source_fresh"] is True


def test_generic_ros2_status_is_structurally_unavailable_without_backend(monkeypatch):
    monkeypatch.setattr(rt, "topic_subscriber_status", lambda topic: {
        "ok": True,
        "running": False,
        "backend": "none",
        "topic": topic,
        "message_type": "",
        "service_id": f"topic-subscriber:{topic}",
        "messages": [],
        "received": 0,
        "source_fresh": False,
        "error": "",
    })

    result = _NODE_REGISTRY["ROS2"]({"action": "status", "topic": "/scan"})

    assert result["status"]["state"] == "unavailable"
    assert result["status"]["available"] is False
    assert "Install ROS 2" in result["report"]


def test_generic_ros2_once_can_discover_message_type(monkeypatch):
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15: {
        "ok": True,
        "backend": "native",
        "stdout": "std_msgs/msg/String\n",
        "stderr": "",
    })
    captured = {}

    def fake_once(**kwargs):
        captured.update(kwargs)
        return {
            "ok": True,
            "running": False,
            "backend": "native",
            "topic": kwargs["topic"],
            "message_type": kwargs["message_type"],
            "service_id": f"topic-subscriber:{kwargs['topic']}",
            "messages": [{"data": "hello"}],
            "latest": {"data": "hello"},
            "received": 1,
            "last_message_time_ns": 42,
            "age_seconds": 0.0,
            "stale_after_seconds": kwargs["stale_after_seconds"],
            "source_fresh": True,
            "error": "",
        }

    monkeypatch.setattr(rt, "run_topic_subscriber_once", fake_once)
    result = _NODE_REGISTRY["ROS2"]({
        "action": "once",
        "topic": "/events",
        "message_type": "",
        "timeout": 3.0,
    })

    assert captured["message_type"] == "std_msgs/msg/String"
    assert captured["public_node_type"] == "ROS2"
    assert result["message"] == {"data": "hello"}
    assert result["status"]["state"] == "ready"


def test_generic_ros2_stop_is_idempotent(monkeypatch):
    monkeypatch.setattr(rt, "stop_topic_subscriber", lambda topic: {
        "ok": True,
        "running": False,
        "backend": "native",
        "topic": topic,
        "message_type": "sensor_msgs/msg/LaserScan",
        "service_id": f"topic-subscriber:{topic}",
        "messages": [],
        "received": 0,
        "source_fresh": False,
        "error": "",
    })

    result = _NODE_REGISTRY["ROS2"]({"action": "stop", "topic": "/scan"})

    assert result["running"] is False
    assert result["status"]["state"] == "stopped"
    assert result["stream"]["topic"] == "/scan"


def test_topic_relay_starts_type_preserving_managed_service(monkeypatch):
    captured = {}
    monkeypatch.setattr(
        rt,
        "detect_backend",
        lambda refresh=False: {"backend": "native", "detail": "test"},
    )

    def fake_start(**kwargs):
        captured.update(kwargs)
        return {"ok": True, "backend": "native"}

    monkeypatch.setattr(rt, "start_topic_relay", fake_start)

    result = _NODE_REGISTRY["ROS2TopicRelay"]({
        "run_id": "front lidar",
        "source_topic": "/scan_raw",
        "destination_topic": "/robot/front_scan",
        "msg_type": "sensor_msgs/msg/LaserScan",
        "qos": "sensor_data",
        "queue_depth": 5,
    })

    assert captured == {
        "run_id": "front_lidar",
        "source_topic": "/scan_raw",
        "destination_topic": "/robot/front_scan",
        "message_type": "sensor_msgs/msg/LaserScan",
        "qos": "sensor_data",
        "queue_depth": 5,
    }
    assert result["running"] is True
    assert "relaying /scan_raw -> /robot/front_scan" in result["report"]


@pytest.mark.parametrize(
    "destination",
    ["/cmd_vel", "/follower/joint_commands", "/arm/trajectory"],
)
def test_topic_relay_blocks_motion_destinations(monkeypatch, destination):
    monkeypatch.setattr(
        rt,
        "detect_backend",
        lambda refresh=False: {"backend": "native", "detail": "test"},
    )
    monkeypatch.setattr(
        rt,
        "start_topic_relay",
        lambda **kwargs: pytest.fail("motion destination must not start a generic relay"),
    )

    result = _NODE_REGISTRY["ROS2TopicRelay"]({
        "source_topic": "/leader/joint_states",
        "destination_topic": destination,
        "msg_type": "sensor_msgs/msg/JointState",
    })

    assert result["running"] is False
    assert "BLOCKED" in result["report"]
    assert "safety-gated" in result["report"]


def test_topic_relay_stop_is_scoped_to_run_id(monkeypatch):
    captured = []
    monkeypatch.setattr(
        rt,
        "detect_backend",
        lambda refresh=False: {"backend": "docker", "detail": "test"},
    )
    monkeypatch.setattr(
        rt,
        "stop_topic_relay",
        lambda run_id: captured.append(run_id) or {
            "ok": True,
            "backend": "docker",
            "stopped": 1,
        },
    )

    result = _NODE_REGISTRY["ROS2TopicRelay"]({
        "action": "stop",
        "run_id": "front lidar",
    })

    assert captured == ["front_lidar"]
    assert result["running"] is False
    assert result["backend"] == "docker"


def test_python_node_starts_file_with_parsed_arguments(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "test"})

    def fake_start(**kwargs):
        captured.update(kwargs)
        return {
            "ok": True,
            "running": True,
            "backend": "docker",
            "run_id": "tutorial_node",
            "script": "tutorials/ros2/my_first_standalone_node.py",
        }

    monkeypatch.setattr(rt, "start_ros2_python_node", fake_start)
    result = _NODE_REGISTRY["ROS2PythonNode"]({
        "run_id": "tutorial node",
        "source_mode": "file",
        "script_path": "tutorials/ros2/my_first_standalone_node.py",
        "arguments": "--robot 'front arm'",
    })

    assert captured == {
        "run_id": "tutorial_node",
        "source_mode": "file",
        "script_path": "tutorials/ros2/my_first_standalone_node.py",
        "code": "",
        "arguments": ["--robot", "front arm"],
    }
    assert result["running"] is True
    assert result["backend"] == "docker"
    assert result["run_id"] == "tutorial_node"


def test_python_node_starts_inline_code(monkeypatch):
    captured = {}
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(
        rt,
        "start_ros2_python_node",
        lambda **kwargs: captured.update(kwargs) or {
            "ok": True,
            "running": True,
            "backend": "native",
            "run_id": "inline_node",
            "script": "inline code",
        },
    )
    result = _NODE_REGISTRY["ROS2PythonNode"]({
        "run_id": "inline_node",
        "source_mode": "inline",
        "code": "import rclpy\n",
    })

    assert captured["source_mode"] == "inline"
    assert captured["code"] == "import rclpy\n"
    assert result["running"] is True
    assert result["script"] == "inline code"


def test_python_node_stop_is_scoped_to_run_id(monkeypatch):
    captured = []
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "test"})
    monkeypatch.setattr(
        rt,
        "stop_ros2_python_node",
        lambda run_id: captured.append(run_id) or {"ok": True, "backend": "docker", "stopped": 1},
    )

    result = _NODE_REGISTRY["ROS2PythonNode"]({"action": "stop", "run_id": "tutorial node"})

    assert captured == ["tutorial_node"]
    assert result["running"] is False
    assert result["run_id"] == "tutorial_node"


def test_python_node_requires_selected_source(monkeypatch):
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "native", "detail": "test"})
    monkeypatch.setattr(
        rt,
        "start_ros2_python_node",
        lambda **kwargs: pytest.fail("missing source must not start a process"),
    )

    file_result = _NODE_REGISTRY["ROS2PythonNode"]({"source_mode": "file", "script_path": ""})
    inline_result = _NODE_REGISTRY["ROS2PythonNode"]({"source_mode": "inline", "code": ""})

    assert file_result["running"] is False
    assert "script_path" in file_result["report"]
    assert inline_result["running"] is False
    assert "enter code" in inline_result["report"]


def test_python_node_resolves_workspace_relative_file_from_editor_server(monkeypatch, tmp_path):
    workspace = tmp_path / "workspace"
    server_dir = workspace / "editor-server"
    script = workspace / "tutorials" / "ros2" / "node.py"
    server_dir.mkdir(parents=True)
    script.parent.mkdir(parents=True)
    script.write_text("print('hello')\n", encoding="utf-8")
    monkeypatch.chdir(server_dir)

    resolved, error = rt._python_node_source(
        source_mode="file",
        script_path="tutorials/ros2/node.py",
        code="",
        run_id="tutorial",
    )

    assert error == ""
    assert resolved == script.resolve()


def test_workspace_build_resolves_editor_relative_path_and_reports_logs(monkeypatch, tmp_path):
    workspace_root = tmp_path / "workspace"
    server_dir = workspace_root / "editor-server"
    ros_workspace = workspace_root / "tutorials" / "ros2" / "lesson_04_workspace"
    package_dir = ros_workspace / "src" / "lesson_04_package"
    server_dir.mkdir(parents=True)
    package_dir.mkdir(parents=True)
    (package_dir / "package.xml").write_text("<package/>", encoding="utf-8")
    monkeypatch.chdir(server_dir)

    captured = {}

    def fake_build(workspace_path, *, packages_select=None, timeout=300.0):
        captured.update(
            workspace_path=workspace_path,
            packages_select=packages_select,
            timeout=timeout,
        )
        return {
            "ok": True,
            "backend": "docker",
            "workspace_path": str(ros_workspace.resolve()),
            "setup_path": "/tmp/workspace/install/setup.bash",
            "stdout": "Starting >>> lesson_04_package\nFinished <<< lesson_04_package",
            "stderr": "",
        }

    monkeypatch.setattr(rt, "build_ros2_workspace", fake_build)
    result = _NODE_REGISTRY["ROS2WorkspaceBuild"]({
        "workspace_path": "tutorials/ros2/lesson_04_workspace",
        "packages_select": "lesson_04_package",
        "timeout": 120,
    })

    assert result["built"] is True
    assert result["backend"] == "docker"
    assert result["workspace_path"] == str(ros_workspace.resolve())
    assert result["logs"][-1] == "Finished <<< lesson_04_package"
    assert captured == {
        "workspace_path": "tutorials/ros2/lesson_04_workspace",
        "packages_select": ["lesson_04_package"],
        "timeout": 120.0,
    }


def test_workspace_path_resolution_requires_colcon_workspace_shape(monkeypatch, tmp_path):
    workspace_root = tmp_path / "workspace"
    server_dir = workspace_root / "editor-server"
    package_dir = workspace_root / "tutorials" / "ros2_ws" / "src" / "demo"
    server_dir.mkdir(parents=True)
    package_dir.mkdir(parents=True)
    (package_dir / "package.xml").write_text("<package/>", encoding="utf-8")
    monkeypatch.chdir(server_dir)

    resolved, error = rt.resolve_workspace_path("tutorials/ros2_ws")

    assert error == ""
    assert resolved == (workspace_root / "tutorials" / "ros2_ws").resolve()


def test_launch_builds_ros2_launch_command(monkeypatch):
    captured = {}

    def fake_managed(key, args):
        captured["key"] = key
        captured["args"] = args
        return {"ok": True, "backend": "native"}

    monkeypatch.setattr(rt, "run_ros2_managed", fake_managed)
    result = _NODE_REGISTRY["ROS2Launch"]({
        "run_id": "front camera",
        "package": "camera_bringup",
        "launch_file": "camera.launch.py",
        "arguments": "device:=0 view:=false",
    })
    assert result["launched"] is True
    assert result["run_id"] == "front_camera"
    assert captured["key"] == "front_camera"
    assert captured["args"] == [
        "launch",
        "camera_bringup",
        "camera.launch.py",
        "device:=0",
        "view:=false",
    ]
    assert "launch running" in result["report"]


def test_launch_forwards_local_workspace_overlay(monkeypatch):
    captured = {}

    def fake_managed(key, args, **kwargs):
        captured.update(key=key, args=args, kwargs=kwargs)
        return {"ok": True, "backend": "docker"}

    monkeypatch.setattr(rt, "run_ros2_managed", fake_managed)
    result = _NODE_REGISTRY["ROS2Launch"]({
        "run_id": "lesson_launch",
        "package": "lesson_04_package",
        "launch_file": "lesson.launch.py",
        "workspace_path": "tutorials/ros2/lesson_04_workspace",
    })

    assert result["launched"] is True
    assert captured["kwargs"] == {
        "workspace_path": "tutorials/ros2/lesson_04_workspace",
    }


def test_launch_stop_is_scoped_to_managed_run(monkeypatch):
    captured = {}

    def fake_stop(key, pattern=""):
        captured.update(key=key, pattern=pattern)
        return {"ok": True, "backend": "native", "stopped": 1}

    monkeypatch.setattr(rt, "stop_ros2_managed", fake_stop)
    result = _NODE_REGISTRY["ROS2Launch"]({
        "action": "stop",
        "run_id": "front camera",
        "package": "camera_bringup",
    })

    assert result["launched"] is False
    assert result["run_id"] == "front_camera"
    assert captured == {
        "key": "front_camera",
        "pattern": "ros2 launch camera_bringup",
    }


def test_topic_interface_inspection_reports_rgbd_publishers(monkeypatch):
    def fake_run(args, timeout=15.0):
        if args == ["topic", "list", "-t"]:
            return {
                "ok": True,
                "backend": "native",
                "stdout": (
                    "/depth_cam/rgb0/image_raw [sensor_msgs/msg/Image]\n"
                    "/depth_cam/depth0/image_raw [sensor_msgs/msg/Image]\n"
                ),
                "stderr": "",
            }
        if args[:2] == ["topic", "info"]:
            return {
                "ok": True,
                "backend": "native",
                "stdout": "Publisher count: 1\nSubscription count: 0\n",
                "stderr": "",
            }
        raise AssertionError(args)

    monkeypatch.setattr(rt, "run_ros2", fake_run)
    result = rt.inspect_topic_interfaces([
        {
            "name": "rgb",
            "topic": "/depth_cam/rgb0/image_raw",
            "message_type": "sensor_msgs/msg/Image",
            "required": True,
        },
        {
            "name": "depth",
            "topic": "/depth_cam/depth0/image_raw",
            "message_type": "sensor_msgs/msg/Image",
            "required": True,
        },
    ])

    assert result["ok"] is True
    assert result["ready"] is True
    assert [item["status"] for item in result["interfaces"]] == [
        "publishing",
        "publishing",
    ]


def test_topic_interface_inspection_identifies_missing_required_topic(monkeypatch):
    monkeypatch.setattr(rt, "run_ros2", lambda args, timeout=15.0: {
        "ok": True,
        "backend": "native",
        "stdout": "/depth_cam/rgb0/image_raw [sensor_msgs/msg/Image]\n",
        "stderr": "",
    })

    result = rt.inspect_topic_interfaces([{
        "name": "depth",
        "topic": "/depth_cam/depth0/image_raw",
        "message_type": "sensor_msgs/msg/Image",
        "required": True,
    }])

    assert result["ready"] is False
    assert result["missing"] == ["/depth_cam/depth0/image_raw"]
    assert result["interfaces"][0]["status"] == "missing"


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


def test_run_forwards_local_workspace_overlay(monkeypatch):
    captured = {}

    def fake_managed(key, args, **kwargs):
        captured.update(key=key, args=args, kwargs=kwargs)
        return {"ok": True, "backend": "docker"}

    monkeypatch.setattr(rt, "run_ros2_managed", fake_managed)
    result = _NODE_REGISTRY["ROS2Run"]({
        "run_id": "lesson_04_publisher",
        "package": "lesson_04_package",
        "executable": "publisher",
        "workspace_path": "tutorials/ros2/lesson_04_workspace",
    })

    assert result["running"] is True
    assert captured["kwargs"] == {
        "workspace_path": "tutorials/ros2/lesson_04_workspace",
    }


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


def test_build_ros2_workspace_copies_sources_and_runs_colcon_in_docker(monkeypatch, tmp_path):
    workspace = tmp_path / "lesson_ws"
    package = workspace / "src" / "lesson_package"
    package.mkdir(parents=True)
    (package / "package.xml").write_text("<package/>", encoding="utf-8")
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "x"})
    monkeypatch.setattr(rt, "ensure_container", lambda: None)
    monkeypatch.setattr(rt, "_ensure_container_colcon", lambda: None)
    copied = {}
    monkeypatch.setattr(
        rt,
        "_copy_to_container",
        lambda host, destination: copied.update(host=host, destination=destination),
    )
    calls = []

    def fake_run(cmd, timeout):
        calls.append((cmd, timeout))
        stdout = "Finished <<< lesson_package" if "colcon build" in cmd[-1] else ""
        return SimpleNamespace(returncode=0, stdout=stdout, stderr="")

    monkeypatch.setattr(rt, "_run", fake_run)
    result = rt.build_ros2_workspace(
        str(workspace),
        packages_select=["lesson_package"],
        timeout=90,
    )

    assert result["ok"] is True
    assert result["backend"] == "docker"
    assert copied["host"] == workspace / "src"
    assert copied["destination"].startswith("/tmp/blacknode_ros2_workspace_lesson_ws_")
    build_shells = [cmd[-1] for cmd, _timeout in calls if "colcon build" in cmd[-1]]
    assert len(build_shells) == 1
    assert "--packages-select lesson_package" in build_shells[0]
    assert result["setup_path"].endswith("/install/setup.bash")


def test_run_ros2_managed_sources_built_docker_workspace(monkeypatch, tmp_path):
    workspace = tmp_path / "lesson_ws"
    package = workspace / "src" / "lesson_package"
    package.mkdir(parents=True)
    (package / "package.xml").write_text("<package/>", encoding="utf-8")
    monkeypatch.setattr(rt, "detect_backend", lambda refresh=False: {"backend": "docker", "detail": "x"})
    monkeypatch.setattr(rt, "ensure_container", lambda: None)
    monkeypatch.setattr(rt, "stop_ros2_managed", lambda key, pattern="": {"ok": True, "stopped": 0})
    calls = []

    def fake_run(cmd, timeout):
        calls.append(cmd)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(rt, "_run", fake_run)
    result = rt.run_ros2_managed(
        "lesson_publisher",
        ["run", "lesson_package", "publisher"],
        workspace_path=str(workspace),
    )

    assert result["ok"] is True
    detached = next(cmd for cmd in calls if cmd[:3] == ["docker", "exec", "-d"])
    assert "source /tmp/blacknode_ros2_workspace_lesson_ws_" in detached[-1]
    assert "exec ros2 run lesson_package publisher" in detached[-1]
    assert not any("apt-get install" in cmd[-1] for cmd in calls)


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


def test_rosbridge_string_subscription_delivers_and_closes(monkeypatch):
    received = []
    events = []

    class FakeTopic:
        def __init__(self, ros, topic, message_type):
            events.append(("topic", topic, message_type))

        def subscribe(self, callback):
            self.callback = callback
            events.append(("subscribe",))

        def unsubscribe(self):
            events.append(("unsubscribe",))

    monkeypatch.setattr(rb, "get_connection", lambda *args, **kwargs: object())
    monkeypatch.setattr(rb, "roslibpy", SimpleNamespace(Topic=FakeTopic))

    session = rb.acquire_string_subscription(
        "127.0.0.1",
        9090,
        "/control",
        received.append,
    )
    session._topic.callback({"data": '{"armed": false}'})
    rb.release_string_subscription(session)

    assert received == ['{"armed": false}']
    assert events[-1] == ("unsubscribe",)


def test_rosbridge_motion_stream_publishes_controller_profile_samples(monkeypatch):
    published = []

    class FakeTopic:
        def __init__(self, ros, topic, message_type):
            pass

        def advertise(self):
            pass

        def publish(self, message):
            published.append(message)

        def unadvertise(self):
            pass

    ros = SimpleNamespace(is_connected=True)
    monkeypatch.setattr(rb, "get_connection", lambda *a, **k: ros)
    monkeypatch.setattr(rb, "roslibpy", SimpleNamespace(Topic=FakeTopic, Message=lambda value: value))
    monkeypatch.setattr(rb.time, "sleep", lambda seconds: None)

    result = rb.stream_motion(
        "127.0.0.1",
        9090,
        "/joint_commands",
        ["joint"],
        {"joint": 0.0},
        {"joint": 1.0},
        ramp_seconds=1.0,
        hold_seconds=0.0,
        rate_hz=10.0,
        alphas=[0.0, 0.1, 0.4, 1.0],
    )

    assert result == {"ok": True, "sent": 4}
    assert [message["position"][0] for message in published] == [0.0, 0.1, 0.4, 1.0]


def test_rosbridge_motion_stream_rejects_unsafe_profile_samples(monkeypatch):
    monkeypatch.setattr(rb, "get_connection", lambda *a, **k: SimpleNamespace(is_connected=True))

    result = rb.stream_motion(
        "127.0.0.1",
        9090,
        "/joint_commands",
        ["joint"],
        {"joint": 0.0},
        {"joint": 1.0},
        ramp_seconds=1.0,
        hold_seconds=0.0,
        rate_hz=10.0,
        alphas=[0.0, 0.8, 0.6, 1.0],
    )

    assert result == {
        "ok": False,
        "sent": 0,
        "error": "invalid normalized motion-profile samples",
    }


def test_joint_stream_seed_config_replaces_stale_torque_state():
    session = rb.JointStreamSession.__new__(rb.JointStreamSession)
    session._data_lock = threading.Lock()
    session._config_event = threading.Event()
    session._config = {"torque_enabled": True, "mode": "hold"}

    session.seed_config({"torque_enabled": False, "mode": "teach"})

    assert session.wait_for_config(0) == {"torque_enabled": False, "mode": "teach"}


def test_rosbridge_read_only_joint_stream_does_not_advertise_commands(monkeypatch):
    events = []

    class FakeTopic:
        def __init__(self, ros, topic, message_type):
            self.topic = topic
            events.append(("topic", topic, message_type))

        def subscribe(self, callback):
            self.callback = callback
            events.append(("subscribe", self.topic))

        def unsubscribe(self):
            events.append(("unsubscribe", self.topic))

        def advertise(self):
            events.append(("advertise", self.topic))

        def unadvertise(self):
            events.append(("unadvertise", self.topic))

    ros = SimpleNamespace(is_connected=True)
    monkeypatch.setattr(rb, "get_connection", lambda *args, **kwargs: ros)
    monkeypatch.setattr(rb, "roslibpy", SimpleNamespace(Topic=FakeTopic))

    session = rb.JointStreamSession(
        "127.0.0.1",
        9090,
        "/leader/joint_states",
        "",
        "/leader/joint_config",
        1.0,
    )

    assert session._command_pub is None
    assert not any(event[0] == "advertise" for event in events)
    with pytest.raises(RuntimeError, match="read-only"):
        session.publish({"joint": 0.5})


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
def test_topic_relay_roundtrip_live():
    publisher = _NODE_REGISTRY["ROS2TopicPublisher"]({
        "action": "start",
        "topic": "/bn_relay_source",
        "payload": "data: relayed",
        "rate_hz": 5.0,
    })
    assert publisher["running"] is True, publisher["report"]
    relay = _NODE_REGISTRY["ROS2TopicRelay"]({
        "action": "start",
        "run_id": "bn_relay_test",
        "source_topic": "/bn_relay_source",
        "destination_topic": "/bn_relay_destination",
        "msg_type": "std_msgs/msg/String",
        "qos": "reliable",
    })
    assert relay["running"] is True, relay["report"]
    try:
        result = _NODE_REGISTRY["ROS2TopicEcho"]({
            "topic": "/bn_relay_destination",
            "msg_type": "std_msgs/msg/String",
            "count": 1,
            "timeout": 30.0,
        })
        assert result["messages"], result["report"]
        assert "relayed" in result["messages"][0]
    finally:
        _NODE_REGISTRY["ROS2TopicRelay"]({
            "action": "stop",
            "run_id": "bn_relay_test",
        })
        _NODE_REGISTRY["ROS2TopicPublisher"]({
            "action": "stop",
            "topic": "/bn_relay_source",
        })


@backend_only
def test_interface_show_live():
    r = _NODE_REGISTRY["ROS2InterfaceShow"]({"interface": "std_msgs/msg/String"})
    assert "string data" in r["definition"], r["report"]
