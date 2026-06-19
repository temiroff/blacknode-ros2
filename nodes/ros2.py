"""ROS 2 nodes for Blacknode.

Topic, service, node, and interface introspection plus publishing, backed by
a native ``ros2`` CLI or a Docker helper container (see ``ros2_runtime``).
Every node returns a structured report instead of raising, so workflows stay
usable on machines without ROS.

The ``trigger`` input is an optional pass-through: wire any upstream port
into it to sequence ROS actions (e.g. start a demo publisher before echoing).
"""
from __future__ import annotations

import base64
import html
import json
import math
import shlex
import time
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

from . import ros2_runtime as rt

_CATEGORY = "ROS 2"


def _report(result: dict[str, Any], action: str) -> str:
    if result.get("ok"):
        return f"{action} OK via {result.get('backend', '?')} backend"
    return f"{action} FAILED: {result.get('error', 'unknown error')}"


@node(
    name="ROS2SystemCheck",
    category=_CATEGORY,
    description="Detect how ROS 2 will run here: native ros2 CLI, Docker container, or unavailable.",
    inputs={"refresh": Bool(default=True)},
    outputs={"available": Bool, "backend": Text, "report": Text},
)
def ros2_system_check(ctx: dict) -> dict:
    info = rt.detect_backend(refresh=bool(ctx.get("refresh", True)))
    backend = info["backend"]
    if backend == "none":
        return {"available": False, "backend": backend, "report": info["detail"]}
    probe = rt.run_ros2(["topic", "list"], timeout=30)
    lines = [
        f"backend: {backend} ({info['detail']})",
        f"ros2 CLI reachable: {'yes' if probe['ok'] else 'no'}",
    ]
    if probe["ok"]:
        topics = [t for t in probe["stdout"].splitlines() if t.strip()]
        lines.append(f"live topics: {len(topics)}")
    else:
        lines.append(f"probe error: {probe.get('error', '')}")
    return {"available": probe["ok"], "backend": backend, "report": "\n".join(lines)}


@node(
    name="ROS2TopicList",
    category=_CATEGORY,
    description="List live ROS 2 topics, optionally with message types.",
    inputs={"trigger": AnyPort, "show_types": Bool(default=True)},
    outputs={"topics": List, "report": Text},
)
def ros2_topic_list(ctx: dict) -> dict:
    args = ["topic", "list"]
    if ctx.get("show_types", True):
        args.append("-t")
    result = rt.run_ros2(args, timeout=30)
    topics = [line.strip() for line in result["stdout"].splitlines() if line.strip()] if result["ok"] else []
    return {"topics": topics, "report": _report(result, "topic list")}


@node(
    name="ROS2TopicEcho",
    category=_CATEGORY,
    description="Read messages from a topic (bounded by count and timeout). Set msg_type to skip type discovery.",
    inputs={
        "trigger": AnyPort,
        "topic": Text(default="/chatter"),
        "msg_type": Text(default=""),
        "count": Int(default=1),
        "timeout": Float(default=10.0),
    },
    outputs={"messages": List, "report": Text},
)
def ros2_topic_echo(ctx: dict) -> dict:
    topic = str(ctx.get("topic") or "/chatter")
    msg_type = str(ctx.get("msg_type") or "").strip()
    count = max(1, int(ctx.get("count") or 1))
    timeout = float(ctx.get("timeout") or 10.0)
    args = ["topic", "echo"]
    if count == 1:
        # clean exit after the first message; --timeout bounds the wait
        args += ["--once", "--timeout", str(max(1, int(timeout)))]
    args += [topic]
    if msg_type:
        args.append(msg_type)
    # count > 1: jazzy's echo has no message-count flag, so stream for the
    # full timeout window and truncate client-side.
    result = rt.run_ros2(args, timeout=timeout)
    messages = [block.strip() for block in result["stdout"].split("---") if block.strip()][:count]
    if result["ok"] or (messages and result.get("timed_out")):
        report = f"received {len(messages)} message(s) from {topic} via {result['backend']}"
        return {"messages": messages, "report": report}
    return {"messages": [], "report": _report(result, f"echo {topic}")}


@node(
    name="ROS2CompressedImageSnapshot",
    category=_CATEGORY,
    description="Capture one sensor_msgs/msg/CompressedImage frame and render it as an image.",
    inputs={
        "trigger": AnyPort,
        "topic": Text(default="/so101/camera/front/compressed"),
        "timeout": Float(default=10.0),
    },
    outputs={"image": Image, "metadata": Dict, "report": Text},
)
def ros2_compressed_image_snapshot(ctx: dict) -> dict:
    topic = str(ctx.get("topic") or "/so101/camera/front/compressed")
    timeout = max(1.0, float(ctx.get("timeout") or 10.0))
    result = rt.run_ros2(
        [
            "topic",
            "echo",
            "--once",
            "--timeout",
            str(max(1, int(timeout))),
            topic,
            "sensor_msgs/msg/CompressedImage",
        ],
        timeout=timeout,
    )
    if not result["ok"]:
        return {"image": "", "metadata": {}, "report": _report(result, f"image snapshot {topic}")}
    try:
        import yaml

        documents = [doc for doc in yaml.safe_load_all(result["stdout"]) if isinstance(doc, dict)]
        message = documents[0] if documents else {}
        raw_data = message.get("data", [])
        if isinstance(raw_data, str):
            image_bytes = base64.b64decode(raw_data)
        else:
            image_bytes = bytes(int(value) for value in raw_data)
        if not image_bytes:
            raise ValueError("message contained no image bytes")
        image_format = str(message.get("format") or "jpeg").lower()
        mime = "image/png" if "png" in image_format else "image/jpeg"
        encoded = base64.b64encode(image_bytes).decode("ascii")
        metadata = {
            "topic": topic,
            "format": image_format,
            "byte_count": len(image_bytes),
            "mime_type": mime,
        }
        return {
            "image": f"data:{mime};base64,{encoded}",
            "metadata": metadata,
            "report": f"captured {len(image_bytes)} byte {image_format} frame from {topic}",
        }
    except Exception as exc:
        return {
            "image": "",
            "metadata": {"topic": topic},
            "report": f"image snapshot {topic} FAILED: could not decode ROS message: {exc}",
        }


@node(
    name="ROS2TopicPublish",
    category=_CATEGORY,
    description="Publish one or more messages to a topic (YAML payload).",
    inputs={
        "trigger": AnyPort,
        "topic": Text(default="/chatter"),
        "msg_type": Text(default="std_msgs/msg/String"),
        "data": Text(default="data: hello from Blacknode"),
        "count": Int(default=1),
    },
    outputs={"report": Text},
)
def ros2_topic_publish(ctx: dict) -> dict:
    topic = str(ctx.get("topic") or "/chatter")
    msg_type = str(ctx.get("msg_type") or "std_msgs/msg/String")
    data = str(ctx.get("data") or "data: hello from Blacknode")
    count = max(1, int(ctx.get("count") or 1))
    args = ["topic", "pub"]
    args += ["--once"] if count == 1 else ["--times", str(count)]
    args += [topic, msg_type, data]
    result = rt.run_ros2(args, timeout=30 + count)
    return {"report": _report(result, f"publish {count}x to {topic}")}


@node(
    name="ROS2DemoPublisher",
    category=_CATEGORY,
    description="Start or stop a background demo publisher so other nodes have a live topic.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "topic": Text(default="/chatter"),
        "message": Text(default="hello from Blacknode"),
        "rate": Float(default=2.0),
    },
    outputs={"report": Text},
)
def ros2_demo_publisher(ctx: dict) -> dict:
    action = str(ctx.get("action") or "start")
    topic = str(ctx.get("topic") or "/chatter")
    if action == "stop":
        result = rt.stop_detached()
        if result["ok"]:
            return {"report": f"stopped {result.get('stopped', 0)} background publisher(s)"}
        return {"report": _report(result, "stop demo publisher")}
    rate = float(ctx.get("rate") or 2.0)
    message = str(ctx.get("message") or "hello from Blacknode")
    result = rt.run_ros2_detached(
        ["topic", "pub", "-r", str(rate), topic, "std_msgs/msg/String", f"data: {message}"]
    )
    if not result["ok"]:
        return {"report": _report(result, "start demo publisher")}
    # Wait until DDS discovery sees the topic, so downstream nodes wired to
    # this report can echo immediately instead of racing discovery.
    deadline = time.time() + 15
    while time.time() < deadline:
        check = rt.run_ros2(["topic", "list"], timeout=10)
        if check["ok"] and topic in check["stdout"].split():
            return {"report": f"demo publisher running on {topic} at {rate:g} Hz via {result['backend']}"}
        time.sleep(1)
    return {"report": f"demo publisher started on {topic} but the topic is not discoverable yet"}


@node(
    name="ROS2NodeList",
    category=_CATEGORY,
    description="List running ROS 2 nodes.",
    inputs={"trigger": AnyPort},
    outputs={"nodes": List, "report": Text},
)
def ros2_node_list(ctx: dict) -> dict:
    result = rt.run_ros2(["node", "list"], timeout=30)
    nodes = [line.strip() for line in result["stdout"].splitlines() if line.strip()] if result["ok"] else []
    return {"nodes": nodes, "report": _report(result, "node list")}


@node(
    name="ROS2ServiceList",
    category=_CATEGORY,
    description="List live ROS 2 services, optionally with types.",
    inputs={"trigger": AnyPort, "show_types": Bool(default=True)},
    outputs={"services": List, "report": Text},
)
def ros2_service_list(ctx: dict) -> dict:
    args = ["service", "list"]
    if ctx.get("show_types", True):
        args.append("-t")
    result = rt.run_ros2(args, timeout=30)
    services = [line.strip() for line in result["stdout"].splitlines() if line.strip()] if result["ok"] else []
    return {"services": services, "report": _report(result, "service list")}


@node(
    name="ROS2InterfaceShow",
    category=_CATEGORY,
    description="Show a message/service definition — lets agents compose valid payloads.",
    inputs={"trigger": AnyPort, "interface": Text(default="std_msgs/msg/String")},
    outputs={"definition": Text, "report": Text},
)
def ros2_interface_show(ctx: dict) -> dict:
    interface = str(ctx.get("interface") or "std_msgs/msg/String")
    result = rt.run_ros2(["interface", "show", interface], timeout=30)
    definition = result["stdout"] if result["ok"] else ""
    return {"definition": definition, "report": _report(result, f"interface show {interface}")}


@node(
    name="ROS2Command",
    category=_CATEGORY,
    description="Escape hatch: run any `ros2 ...` subcommand and capture its output.",
    inputs={"trigger": AnyPort, "args": Text(default="topic list"), "timeout": Float(default=30.0)},
    outputs={"output": Text, "report": Text},
)
def ros2_command(ctx: dict) -> dict:
    raw = str(ctx.get("args") or "").strip()
    if not raw:
        return {"output": "", "report": "ros2 command FAILED: no arguments given"}
    args = shlex.split(raw)
    result = rt.run_ros2(args, timeout=float(ctx.get("timeout") or 30.0))
    output = result["stdout"] if result["ok"] else result.get("error", "")
    return {"output": output, "report": _report(result, f"ros2 {raw}")}


def _svg_text(value: Any, limit: int = 90) -> str:
    text = " ".join(str(value or "").split())
    if len(text) > limit:
        text = text[:limit - 3] + "..."
    return html.escape(text)


def _svg_data(svg: str) -> str:
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


@node(
    name="ROS2VisualDashboard",
    category=_CATEGORY,
    description="Render ROS 2 roundtrip results as a visual pass/fail dashboard.",
    inputs={
        "status": Text,
        "publisher": Text,
        "echo_report": Text,
        "messages": List,
        "topics": List,
        "nodes": List,
        "services": List,
        "definition": Text,
        "expected_topic": Text(default="/blacknode_demo"),
        "expected_message": Text(default="Blacknode ROS 2 roundtrip works"),
    },
    outputs={"dashboard": Image, "passed": Bool, "summary": Dict},
)
def ros2_visual_dashboard(ctx: dict) -> dict:
    status = str(ctx.get("status") or "")
    publisher = str(ctx.get("publisher") or "")
    echo_report = str(ctx.get("echo_report") or "")
    messages = list(ctx.get("messages") or [])
    topics = list(ctx.get("topics") or [])
    nodes = list(ctx.get("nodes") or [])
    services = list(ctx.get("services") or [])
    definition = str(ctx.get("definition") or "")
    expected_topic = str(ctx.get("expected_topic") or "/blacknode_demo")
    expected_message = str(ctx.get("expected_message") or "Blacknode ROS 2 roundtrip works")

    message_text = "\n".join(str(item) for item in messages)
    topics_text = "\n".join(str(item) for item in topics)
    backend_ok = "ros2 CLI reachable: yes" in status
    publisher_ok = "running on" in publisher and "FAILED" not in publisher
    message_ok = expected_message in message_text
    topic_ok = expected_topic in topics_text
    passed = backend_ok and publisher_ok and message_ok and topic_ok

    backend = "unavailable"
    for line in status.splitlines():
        if line.lower().startswith("backend:"):
            backend = line.split(":", 1)[1].strip()
            break

    summary = {
        "passed": passed,
        "backend": backend,
        "publisher_ok": publisher_ok,
        "message_ok": message_ok,
        "topic_ok": topic_ok,
        "topic_count": len(topics),
        "node_count": len(nodes),
        "service_count": len(services),
        "expected_topic": expected_topic,
        "expected_message": expected_message,
    }

    verdict = "PASS" if passed else "FAIL"
    accent = "#22c55e" if passed else "#ef4444"
    muted = "#93a4b8"
    panel = "#172033"
    message_display = messages[0] if messages else echo_report or "No message captured"
    interface_display = "string data" if "string data" in definition else (
        definition.splitlines()[-1] if definition.splitlines() else "definition unavailable"
    )

    def check_mark(ok: bool) -> str:
        return "PASS" if ok else "FAIL"

    def check_color(ok: bool) -> str:
        return "#22c55e" if ok else "#ef4444"

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="650" viewBox="0 0 1120 650">
<rect width="1120" height="650" rx="28" fill="#0b1020"/>
<rect x="24" y="24" width="1072" height="82" rx="18" fill="{panel}" stroke="#2e9fe6" stroke-width="2"/>
<circle cx="66" cy="65" r="18" fill="#2e9fe6"/><circle cx="66" cy="65" r="8" fill="#0b1020"/>
<text x="100" y="58" fill="#f8fafc" font-family="Arial,sans-serif" font-size="26" font-weight="700">ROS 2 LIVE ROUNDTRIP</text>
<text x="100" y="83" fill="{muted}" font-family="Arial,sans-serif" font-size="15">Blacknode visual integration test</text>
<rect x="930" y="42" width="132" height="46" rx="23" fill="{accent}"/>
<text x="996" y="72" text-anchor="middle" fill="#ffffff" font-family="Arial,sans-serif" font-size="22" font-weight="800">{verdict}</text>

<text x="36" y="140" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">MESSAGE PATH</text>
<rect x="36" y="160" width="190" height="88" rx="14" fill="{panel}" stroke="#2e9fe6"/>
<text x="131" y="193" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="17" font-weight="700">BLACKNODE</text>
<text x="131" y="220" text-anchor="middle" fill="{muted}" font-family="Arial,sans-serif" font-size="13">workflow trigger</text>
<path d="M226 204 H276" stroke="#2e9fe6" stroke-width="4"/><path d="M276 204 l-12 -8 v16 z" fill="#2e9fe6"/>
<rect x="284" y="160" width="190" height="88" rx="14" fill="{panel}" stroke="{check_color(publisher_ok)}" stroke-width="2"/>
<text x="379" y="193" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="17" font-weight="700">PUBLISHER</text>
<text x="379" y="220" text-anchor="middle" fill="{check_color(publisher_ok)}" font-family="Arial,sans-serif" font-size="13">{check_mark(publisher_ok)}</text>
<path d="M474 204 H524" stroke="#f59e0b" stroke-width="4"/><path d="M524 204 l-12 -8 v16 z" fill="#f59e0b"/>
<rect x="532" y="160" width="240" height="88" rx="14" fill="{panel}" stroke="#f59e0b"/>
<text x="652" y="193" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="17" font-weight="700">{_svg_text(expected_topic, 30)}</text>
<text x="652" y="220" text-anchor="middle" fill="{check_color(topic_ok)}" font-family="Arial,sans-serif" font-size="13">DISCOVERY {check_mark(topic_ok)}</text>
<path d="M772 204 H822" stroke="#2e9fe6" stroke-width="4"/><path d="M822 204 l-12 -8 v16 z" fill="#2e9fe6"/>
<rect x="830" y="160" width="254" height="88" rx="14" fill="{panel}" stroke="{check_color(message_ok)}" stroke-width="2"/>
<text x="957" y="193" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="17" font-weight="700">ECHO CAPTURE</text>
<text x="957" y="220" text-anchor="middle" fill="{check_color(message_ok)}" font-family="Arial,sans-serif" font-size="13">{check_mark(message_ok)}</text>

<text x="36" y="286" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">LIVE GRAPH</text>
<rect x="36" y="306" width="252" height="108" rx="14" fill="{panel}"/>
<text x="56" y="336" fill="{muted}" font-family="Arial,sans-serif" font-size="13">BACKEND</text>
<text x="56" y="368" fill="#f8fafc" font-family="Arial,sans-serif" font-size="18" font-weight="700">{_svg_text(backend, 28)}</text>
<text x="56" y="394" fill="{check_color(backend_ok)}" font-family="Arial,sans-serif" font-size="13">CLI {check_mark(backend_ok)}</text>
<rect x="306" y="306" width="236" height="108" rx="14" fill="{panel}"/>
<text x="326" y="336" fill="{muted}" font-family="Arial,sans-serif" font-size="13">TOPICS DISCOVERED</text>
<text x="326" y="388" fill="#f97316" font-family="Arial,sans-serif" font-size="42" font-weight="800">{len(topics)}</text>
<rect x="560" y="306" width="236" height="108" rx="14" fill="{panel}"/>
<text x="580" y="336" fill="{muted}" font-family="Arial,sans-serif" font-size="13">ROS NODES</text>
<text x="580" y="388" fill="#22c55e" font-family="Arial,sans-serif" font-size="42" font-weight="800">{len(nodes)}</text>
<rect x="814" y="306" width="270" height="108" rx="14" fill="{panel}"/>
<text x="834" y="336" fill="{muted}" font-family="Arial,sans-serif" font-size="13">ROS SERVICES</text>
<text x="834" y="388" fill="#a855f7" font-family="Arial,sans-serif" font-size="42" font-weight="800">{len(services)}</text>

<rect x="36" y="446" width="1048" height="84" rx="14" fill="{panel}" stroke="{check_color(message_ok)}"/>
<text x="56" y="476" fill="{muted}" font-family="Arial,sans-serif" font-size="13">CAPTURED MESSAGE</text>
<text x="56" y="510" fill="#f8fafc" font-family="monospace" font-size="19" font-weight="700">{_svg_text(message_display, 96)}</text>

<rect x="36" y="550" width="1048" height="66" rx="14" fill="{panel}"/>
<text x="56" y="578" fill="{muted}" font-family="Arial,sans-serif" font-size="13">INTERFACE</text>
<text x="56" y="602" fill="#f8fafc" font-family="monospace" font-size="16">std_msgs/msg/String  -  {_svg_text(interface_display, 70)}</text>
</svg>"""
    return {
        "dashboard": _svg_data(svg),
        "passed": passed,
        "summary": summary,
    }


_SO101_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


@node(
    name="SO101ROS2BridgePlan",
    category=_CATEGORY,
    description="Render a safety-gated SO-ARM101 LeRobot-to-ROS 2 bridge architecture and launch command.",
    inputs={
        "serial_port": Text(default="COM3"),
        "robot_id": Text(default="my_so101"),
        "camera_index": Int(default=0),
        "camera_name": Text(default="front"),
        "max_relative_target": Float(default=5.0),
        "motion_enabled": Bool(default=False),
    },
    outputs={
        "architecture": Image,
        "config": Dict,
        "launch_command": Text,
        "checklist": List,
    },
)
def so101_ros2_bridge_plan(ctx: dict) -> dict:
    serial_port = str(ctx.get("serial_port") or "COM3")
    robot_id = str(ctx.get("robot_id") or "my_so101")
    camera_index = int(ctx.get("camera_index") if ctx.get("camera_index") is not None else 0)
    camera_name = str(ctx.get("camera_name") or "front")
    max_relative_target = max(0.1, float(ctx.get("max_relative_target") or 5.0))
    motion_enabled = bool(ctx.get("motion_enabled", False))
    namespace = "/so101"

    topics = {
        "state_native": f"{namespace}/state",
        "joint_states": "/joint_states",
        "image": f"{namespace}/camera/{camera_name}/compressed",
        "status": f"{namespace}/status",
        "command": f"{namespace}/command",
        "enable": f"{namespace}/enable",
        "stop": f"{namespace}/stop",
    }
    config = {
        "serial_port": serial_port,
        "robot_id": robot_id,
        "camera_index": camera_index,
        "camera_name": camera_name,
        "max_relative_target": max_relative_target,
        "motion_enabled": motion_enabled,
        "joint_order": list(_SO101_JOINTS),
        "command_units": "degrees for arm joints; 0-100 for gripper",
        "topics": topics,
    }
    command = (
        "python packages/blacknode-ros2/scripts/so101_ros2_bridge.py"
        f" --port {json.dumps(serial_port)}"
        f" --robot-id {json.dumps(robot_id)}"
        f" --camera-index {camera_index}"
        f" --camera-name {json.dumps(camera_name)}"
        f" --max-relative-target {max_relative_target:g}"
    )
    if motion_enabled:
        command += " --enable-motion"

    checklist = [
        "Run lerobot-find-port and replace the serial-port placeholder.",
        "Complete SO-101 motor setup and calibration using the same robot id.",
        "Start the bridge without --enable-motion and verify state, status, and camera topics.",
        "Keep a physical power cutoff within reach; /so101/stop is only a software latch.",
        "For motion, add --enable-motion, publish true to /so101/enable, then arm the Blacknode publisher.",
    ]

    gate_label = "MOTION CAPABLE" if motion_enabled else "OBSERVATION ONLY"
    gate_color = "#f59e0b" if motion_enabled else "#22c55e"
    camera_label = "disabled" if camera_index < 0 else f"camera {camera_index}"
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="1180" height="690" viewBox="0 0 1180 690">
<rect width="1180" height="690" rx="28" fill="#09111f"/>
<rect x="24" y="24" width="1132" height="82" rx="18" fill="#162238" stroke="#2e9fe6" stroke-width="2"/>
<text x="54" y="60" fill="#f8fafc" font-family="Arial,sans-serif" font-size="27" font-weight="700">SO-ARM101 ROS 2 CONTROL PLAN</text>
<text x="54" y="86" fill="#93a4b8" font-family="Arial,sans-serif" font-size="14">LeRobot owns the Feetech bus; ROS 2 exposes observations and guarded commands.</text>
<rect x="918" y="43" width="210" height="45" rx="22" fill="{gate_color}"/>
<text x="1023" y="72" text-anchor="middle" fill="#07111e" font-family="Arial,sans-serif" font-size="15" font-weight="800">{gate_label}</text>

<text x="36" y="142" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">DATA AND CONTROL PATH</text>
<rect x="36" y="165" width="212" height="154" rx="16" fill="#162238" stroke="#f97316"/>
<text x="142" y="198" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="19" font-weight="700">SO-ARM101</text>
<text x="56" y="229" fill="#93a4b8" font-family="monospace" font-size="13">Feetech serial: {_svg_text(serial_port, 18)}</text>
<text x="56" y="254" fill="#93a4b8" font-family="monospace" font-size="13">6 motors / calibrated</text>
<text x="56" y="279" fill="#93a4b8" font-family="monospace" font-size="13">{_svg_text(camera_label, 22)}</text>
<text x="56" y="304" fill="#93a4b8" font-family="monospace" font-size="13">LeRobot native units</text>

<path d="M248 242 H300" stroke="#f97316" stroke-width="4"/><path d="M300 242 l-12 -8 v16 z" fill="#f97316"/>
<rect x="308" y="165" width="224" height="154" rx="16" fill="#162238" stroke="#a855f7"/>
<text x="420" y="198" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="19" font-weight="700">LEROBOT</text>
<text x="328" y="229" fill="#93a4b8" font-family="monospace" font-size="13">get_observation()</text>
<text x="328" y="254" fill="#93a4b8" font-family="monospace" font-size="13">send_action()</text>
<text x="328" y="279" fill="#93a4b8" font-family="monospace" font-size="13">relative limit: {max_relative_target:g}</text>
<text x="328" y="304" fill="#93a4b8" font-family="monospace" font-size="13">robot id: {_svg_text(robot_id, 18)}</text>

<path d="M532 242 H584" stroke="#a855f7" stroke-width="4"/><path d="M584 242 l-12 -8 v16 z" fill="#a855f7"/>
<rect x="592" y="145" width="260" height="194" rx="16" fill="#162238" stroke="#2e9fe6" stroke-width="2"/>
<text x="722" y="181" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="19" font-weight="700">ROS 2 BRIDGE</text>
<text x="612" y="214" fill="#22c55e" font-family="monospace" font-size="13">PUB  /so101/state</text>
<text x="612" y="239" fill="#22c55e" font-family="monospace" font-size="13">PUB  /joint_states</text>
<text x="612" y="264" fill="#22c55e" font-family="monospace" font-size="13">PUB  /so101/camera/.../compressed</text>
<text x="612" y="289" fill="#f59e0b" font-family="monospace" font-size="13">SUB  /so101/command</text>
<text x="612" y="314" fill="#ef4444" font-family="monospace" font-size="13">SUB  /so101/enable + /stop</text>

<path d="M852 242 H904" stroke="#2e9fe6" stroke-width="4"/><path d="M904 242 l-12 -8 v16 z" fill="#2e9fe6"/>
<rect x="912" y="165" width="232" height="154" rx="16" fill="#162238" stroke="#22c55e"/>
<text x="1028" y="198" text-anchor="middle" fill="#f8fafc" font-family="Arial,sans-serif" font-size="19" font-weight="700">BLACKNODE</text>
<text x="932" y="229" fill="#93a4b8" font-family="monospace" font-size="13">image viewer</text>
<text x="932" y="254" fill="#93a4b8" font-family="monospace" font-size="13">joint preview</text>
<text x="932" y="279" fill="#93a4b8" font-family="monospace" font-size="13">guarded publish</text>
<text x="932" y="304" fill="#93a4b8" font-family="monospace" font-size="13">run history</text>

<text x="36" y="382" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">THREE MOTION GATES</text>
<rect x="36" y="404" width="350" height="104" rx="15" fill="#162238"/>
<circle cx="72" cy="440" r="19" fill="#f59e0b"/><text x="72" y="447" text-anchor="middle" fill="#07111e" font-family="Arial" font-size="18" font-weight="800">1</text>
<text x="108" y="437" fill="#f8fafc" font-family="Arial" font-size="16" font-weight="700">Bridge launch flag</text>
<text x="108" y="466" fill="#93a4b8" font-family="monospace" font-size="13">--enable-motion</text>
<rect x="415" y="404" width="350" height="104" rx="15" fill="#162238"/>
<circle cx="451" cy="440" r="19" fill="#ef4444"/><text x="451" y="447" text-anchor="middle" fill="#ffffff" font-family="Arial" font-size="18" font-weight="800">2</text>
<text x="487" y="437" fill="#f8fafc" font-family="Arial" font-size="16" font-weight="700">Runtime ROS enable</text>
<text x="487" y="466" fill="#93a4b8" font-family="monospace" font-size="13">/so101/enable = true</text>
<rect x="794" y="404" width="350" height="104" rx="15" fill="#162238"/>
<circle cx="830" cy="440" r="19" fill="#2e9fe6"/><text x="830" y="447" text-anchor="middle" fill="#07111e" font-family="Arial" font-size="18" font-weight="800">3</text>
<text x="866" y="437" fill="#f8fafc" font-family="Arial" font-size="16" font-weight="700">Blacknode command arm</text>
<text x="866" y="466" fill="#93a4b8" font-family="monospace" font-size="13">armed = true</text>

<rect x="36" y="540" width="1108" height="112" rx="16" fill="#111b2d" stroke="#334155"/>
<text x="56" y="572" fill="#93a4b8" font-family="Arial" font-size="13" font-weight="700">GENERATED BRIDGE COMMAND</text>
<text x="56" y="605" fill="#f8fafc" font-family="monospace" font-size="14">{_svg_text(command, 125)}</text>
<text x="56" y="633" fill="#ef4444" font-family="Arial" font-size="13">A software stop does not replace a physical power cutoff or a cleared workspace.</text>
</svg>"""
    return {
        "architecture": _svg_data(svg),
        "config": config,
        "launch_command": command,
        "checklist": checklist,
    }


@node(
    name="SO101JointCommandPreview",
    category=_CATEGORY,
    description="Preview an SO-ARM101 pose and build a ROS command without publishing it.",
    inputs={
        "bridge": Dict,
        "shoulder_pan": Float(default=0.0),
        "shoulder_lift": Float(default=-20.0),
        "elbow_flex": Float(default=35.0),
        "wrist_flex": Float(default=15.0),
        "wrist_roll": Float(default=0.0),
        "gripper": Float(default=25.0),
    },
    outputs={"preview": Image, "command": Dict, "payload": Text},
)
def so101_joint_command_preview(ctx: dict) -> dict:
    bridge = dict(ctx.get("bridge") or {})
    values = [
        float(ctx.get("shoulder_pan") or 0.0),
        float(ctx.get("shoulder_lift") or 0.0),
        float(ctx.get("elbow_flex") or 0.0),
        float(ctx.get("wrist_flex") or 0.0),
        float(ctx.get("wrist_roll") or 0.0),
        min(100.0, max(0.0, float(ctx.get("gripper") or 0.0))),
    ]
    topic = str((bridge.get("topics") or {}).get("command") or "/so101/command")
    command = {
        "topic": topic,
        "msg_type": "std_msgs/msg/Float64MultiArray",
        "joint_order": list(_SO101_JOINTS),
        "data": values,
        "units": "degrees for first five joints; 0-100 for gripper",
    }
    payload = json.dumps({"data": values}, separators=(",", ":"))

    origin_x, origin_y = 360.0, 485.0
    lengths = [135.0, 125.0, 82.0]
    angles = [
        math.radians(-90.0 + values[1]),
        math.radians(values[2]),
        math.radians(values[3]),
    ]
    points = [(origin_x, origin_y)]
    angle = angles[0]
    for index, length in enumerate(lengths):
        if index:
            angle += angles[index]
        x = points[-1][0] + length * math.cos(angle)
        y = points[-1][1] + length * math.sin(angle)
        points.append((x, y))
    point_string = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    gripper_gap = 8.0 + values[5] * 0.18
    end_x, end_y = points[-1]
    mode = "MOTION REQUESTED" if bridge.get("motion_enabled") else "PREVIEW ONLY"
    mode_color = "#f59e0b" if bridge.get("motion_enabled") else "#22c55e"

    rows = "".join(
        f'<text x="650" y="{225 + index * 42}" fill="#93a4b8" font-family="monospace" font-size="14">'
        f'{_svg_text(name, 18)}</text><text x="1010" y="{225 + index * 42}" text-anchor="end" '
        f'fill="#f8fafc" font-family="monospace" font-size="16" font-weight="700">{value:.1f}</text>'
        for index, (name, value) in enumerate(zip(_SO101_JOINTS, values))
    )
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="650" viewBox="0 0 1080 650">
<rect width="1080" height="650" rx="28" fill="#09111f"/>
<rect x="24" y="24" width="1032" height="78" rx="18" fill="#162238" stroke="#2e9fe6" stroke-width="2"/>
<text x="52" y="59" fill="#f8fafc" font-family="Arial" font-size="26" font-weight="700">SO-ARM101 JOINT COMMAND PREVIEW</text>
<text x="52" y="84" fill="#93a4b8" font-family="Arial" font-size="14">This node creates a command payload but never sends it.</text>
<rect x="832" y="42" width="196" height="42" rx="21" fill="{mode_color}"/>
<text x="930" y="69" text-anchor="middle" fill="#07111e" font-family="Arial" font-size="14" font-weight="800">{mode}</text>

<rect x="34" y="132" width="562" height="474" rx="18" fill="#111b2d"/>
<text x="56" y="166" fill="#93a4b8" font-family="Arial" font-size="13" font-weight="700">PLANAR POSE APPROXIMATION</text>
<line x1="120" y1="520" x2="545" y2="520" stroke="#334155" stroke-width="2"/>
<rect x="310" y="486" width="100" height="34" rx="8" fill="#334155"/>
<polyline points="{point_string}" fill="none" stroke="#f97316" stroke-width="18" stroke-linecap="round" stroke-linejoin="round"/>
{''.join(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="12" fill="#2e9fe6" stroke="#dbeafe" stroke-width="3"/>' for x, y in points)}
<line x1="{end_x:.1f}" y1="{end_y - gripper_gap:.1f}" x2="{end_x + 38:.1f}" y2="{end_y - gripper_gap:.1f}" stroke="#22c55e" stroke-width="8" stroke-linecap="round"/>
<line x1="{end_x:.1f}" y1="{end_y + gripper_gap:.1f}" x2="{end_x + 38:.1f}" y2="{end_y + gripper_gap:.1f}" stroke="#22c55e" stroke-width="8" stroke-linecap="round"/>
<text x="56" y="574" fill="#93a4b8" font-family="monospace" font-size="13">pan {values[0]:.1f} deg | roll {values[4]:.1f} deg | gripper {values[5]:.1f}%</text>

<rect x="620" y="132" width="426" height="350" rx="18" fill="#111b2d"/>
<text x="650" y="166" fill="#93a4b8" font-family="Arial" font-size="13" font-weight="700">LEROBOT NATIVE COMMAND</text>
<text x="1010" y="166" text-anchor="end" fill="#93a4b8" font-family="Arial" font-size="12">DEG / PERCENT</text>
{rows}
<rect x="620" y="504" width="426" height="102" rx="18" fill="#162238" stroke="#f59e0b"/>
<text x="646" y="537" fill="#93a4b8" font-family="Arial" font-size="13" font-weight="700">ROS 2 TOPIC</text>
<text x="646" y="567" fill="#f8fafc" font-family="monospace" font-size="15">{_svg_text(topic, 38)}</text>
<text x="646" y="590" fill="#f59e0b" font-family="Arial" font-size="12">Publishing requires all three motion gates.</text>
</svg>"""
    return {"preview": _svg_data(svg), "command": command, "payload": payload}


@node(
    name="SO101JointCommandPublish",
    category=_CATEGORY,
    description="Publish a previewed SO-ARM101 command only when this node is explicitly armed.",
    inputs={"trigger": AnyPort, "command": Dict, "armed": Bool(default=False)},
    outputs={"report": Text},
)
def so101_joint_command_publish(ctx: dict) -> dict:
    if not bool(ctx.get("armed", False)):
        return {
            "report": (
                "BLOCKED: command preview only. To permit physical motion, launch the bridge "
                "with --enable-motion, publish true to /so101/enable, and set armed=true here."
            )
        }
    command = dict(ctx.get("command") or {})
    values = list(command.get("data") or [])
    if len(values) != len(_SO101_JOINTS):
        return {"report": "BLOCKED: command must contain exactly six joint values"}
    try:
        values = [float(value) for value in values]
    except (TypeError, ValueError):
        return {"report": "BLOCKED: all joint values must be numeric"}
    if not all(math.isfinite(value) for value in values):
        return {"report": "BLOCKED: joint values must be finite"}
    if not 0.0 <= values[-1] <= 100.0:
        return {"report": "BLOCKED: gripper must be between 0 and 100"}

    topic = str(command.get("topic") or "/so101/command")
    msg_type = str(command.get("msg_type") or "std_msgs/msg/Float64MultiArray")
    payload = json.dumps({"data": values}, separators=(",", ":"))
    result = rt.run_ros2(["topic", "pub", "--once", topic, msg_type, payload], timeout=30)
    return {"report": _report(result, f"SO-101 command publish to {topic}")}
