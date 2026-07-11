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
        "topic": Text(default="/camera/front/compressed"),
        "timeout": Float(default=10.0),
    },
    outputs={"image": Image, "metadata": Dict, "report": Text},
)
def ros2_compressed_image_snapshot(ctx: dict) -> dict:
    topic = str(ctx.get("topic") or "/camera/front/compressed")
    timeout = max(1.0, float(ctx.get("timeout") or 10.0))
    result = rt.capture_image_snapshot(
        topic=topic,
        message_type="compressed",
        timeout=timeout,
        output_format="jpeg",
        jpeg_quality=90,
    )
    if not result["ok"]:
        return {"image": "", "metadata": {}, "report": _report(result, f"image snapshot {topic}")}
    metadata = {"topic": topic, **dict(result.get("metadata") or {})}
    return {
        "image": str(result.get("image") or ""),
        "metadata": metadata,
        "report": f"captured compressed image frame from {topic} ({metadata.get('width', '?')}x{metadata.get('height', '?')})",
    }


@node(
    name="ROS2ImageSnapshot",
    category=_CATEGORY,
    description="Capture one raw sensor_msgs/msg/Image frame and render it as a Blacknode image.",
    inputs={
        "trigger": AnyPort,
        "topic": Text(default="/camera/image_raw"),
        "timeout": Float(default=10.0),
        "output_format": Enum(["png", "jpeg"], default="png"),
        "jpeg_quality": Int(default=90),
    },
    outputs={"image": Image, "metadata": Dict, "report": Text},
)
def ros2_image_snapshot(ctx: dict) -> dict:
    topic = str(ctx.get("topic") or "/camera/image_raw")
    timeout = max(1.0, float(ctx.get("timeout") or 10.0))
    output_format = str(ctx.get("output_format") or "png").strip().lower()
    if output_format not in {"png", "jpeg"}:
        output_format = "png"
    result = rt.capture_image_snapshot(
        topic=topic,
        message_type="raw",
        timeout=timeout,
        output_format=output_format,
        jpeg_quality=int(ctx.get("jpeg_quality") or 90),
    )
    if not result["ok"]:
        return {"image": "", "metadata": {}, "report": _report(result, f"raw image snapshot {topic}")}
    metadata = {"topic": topic, **dict(result.get("metadata") or {})}
    return {
        "image": str(result.get("image") or ""),
        "metadata": metadata,
        "report": (
            f"captured {metadata.get('width', '?')}x{metadata.get('height', '?')} "
            f"{metadata.get('encoding', 'image')} frame from {topic}"
        ),
    }


def _resolve_image_message_type(topic: str, requested: str) -> tuple[str, str]:
    value = requested.strip().lower()
    if value in {"raw", "compressed"}:
        return value, ""
    result = rt.run_ros2(["topic", "type", topic], timeout=10)
    if not result.get("ok"):
        return "", result.get("error", "could not discover topic type")
    types = [line.strip() for line in result.get("stdout", "").splitlines() if line.strip()]
    if any("sensor_msgs/msg/CompressedImage" in line for line in types):
        return "compressed", ""
    if any("sensor_msgs/msg/Image" in line for line in types):
        return "raw", ""
    return "", f"{topic} is not a sensor_msgs Image topic (types: {', '.join(types) or 'none'})"


@node(
    name="ROS2ImageStream",
    category=_CATEGORY,
    description="Start or stop a live MJPEG preview for a raw or compressed ROS 2 image topic.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "stream_id": Text(default="camera"),
        "topic": Text(default="/camera/image_raw"),
        "message_type": Enum(["auto", "raw", "compressed"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=0),
        "max_fps": Float(default=10.0),
        "max_width": Int(default=960),
        "jpeg_quality": Int(default=80),
    },
    outputs={
        "preview": Image,
        "streaming": Bool,
        "stream_url": Text,
        "snapshot_url": Text,
        "stream_id": Text,
        "report": Text,
    },
)
def ros2_image_stream(ctx: dict) -> dict:
    stream_id = str(ctx.get("stream_id") or "camera").strip() or "camera"
    action = str(ctx.get("action") or "start").strip().lower()
    if action == "stop":
        result = rt.stop_image_stream(stream_id)
        return {
            "preview": "",
            "streaming": False,
            "stream_url": "",
            "snapshot_url": "",
            "stream_id": stream_id,
            "report": f"stopped {result.get('stopped', 0)} image stream(s)",
        }

    topic = str(ctx.get("topic") or "/camera/image_raw").strip()
    message_type, error = _resolve_image_message_type(topic, str(ctx.get("message_type") or "auto"))
    if error:
        return {
            "preview": "",
            "streaming": False,
            "stream_url": "",
            "snapshot_url": "",
            "stream_id": stream_id,
            "report": f"image stream FAILED: {error}",
        }

    host = str(ctx.get("host") or "127.0.0.1").strip() or "127.0.0.1"
    port = max(0, int(ctx.get("port") or 0))
    max_fps = max(0.1, min(60.0, float(ctx.get("max_fps") or 10.0)))
    max_width = max(0, int(ctx.get("max_width") or 960))
    jpeg_quality = max(1, min(100, int(ctx.get("jpeg_quality") or 80)))
    result = rt.start_image_stream(
        stream_id=stream_id,
        topic=topic,
        message_type=message_type,
        host=host,
        port=port,
        max_fps=max_fps,
        max_width=max_width,
        jpeg_quality=jpeg_quality,
    )
    if not result.get("ok"):
        return {
            "preview": "",
            "streaming": False,
            "stream_url": "",
            "snapshot_url": "",
            "stream_id": stream_id,
            "report": f"image stream FAILED: {result.get('error', 'unknown error')}",
        }
    stream_url = str(result["stream_url"])
    snapshot_url = str(result["snapshot_url"])
    report = (
        f"LIVE STREAM running on {stream_url} from {topic} "
        f"({message_type}, {max_fps:g} FPS max, width {max_width or 'source'})"
    )
    return {
        "preview": stream_url,
        "streaming": True,
        "stream_url": stream_url,
        "snapshot_url": snapshot_url,
        "stream_id": stream_id,
        "report": report,
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
    name="ROS2Launch",
    category=_CATEGORY,
    description="Start or stop a background `ros2 launch ...` process.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "package": Text(default=""),
        "launch_file": Text(default=""),
        "arguments": Text(default=""),
        "expected_topic": Text(default=""),
        "wait_seconds": Float(default=0.0),
        "stop_pattern": Text(default=""),
    },
    outputs={"launched": Bool, "report": Text},
)
def ros2_launch(ctx: dict) -> dict:
    action = str(ctx.get("action") or "start")
    package = str(ctx.get("package") or "").strip()
    launch_file = str(ctx.get("launch_file") or "").strip()

    if action == "stop":
        pattern = str(ctx.get("stop_pattern") or "").strip() or f"ros2 launch {package}".strip() or "ros2 launch"
        result = rt.stop_detached(pattern=pattern)
        if result["ok"]:
            return {"launched": False, "report": f"stopped {result.get('stopped', 0)} background launch process(es)"}
        return {"launched": False, "report": _report(result, f"stop launch {package}")}

    if not package or not launch_file:
        return {"launched": False, "report": "ros2 launch FAILED: set package and launch_file"}
    try:
        extra_args = shlex.split(str(ctx.get("arguments") or ""))
    except ValueError as exc:
        return {"launched": False, "report": f"ros2 launch FAILED: invalid arguments: {exc}"}

    result = rt.run_ros2_detached(["launch", package, launch_file, *extra_args])
    if not result["ok"]:
        return {"launched": False, "report": _report(result, f"start launch {package} {launch_file}")}

    expected_topic = str(ctx.get("expected_topic") or "").strip()
    wait_seconds = max(0.0, float(ctx.get("wait_seconds") or 0.0))
    if expected_topic and wait_seconds > 0:
        deadline = time.time() + wait_seconds
        while time.time() < deadline:
            check = rt.run_ros2(["topic", "list"], timeout=10)
            topics = {line.strip().split()[0] for line in check.get("stdout", "").splitlines() if line.strip()}
            if check.get("ok") and expected_topic in topics:
                return {
                    "launched": True,
                    "report": (
                        f"launch running: {package} {launch_file}; "
                        f"{expected_topic} is discoverable via {result['backend']} backend"
                    ),
                }
            time.sleep(1)
        return {
            "launched": True,
            "report": f"launch started: {package} {launch_file}, but {expected_topic} was not discoverable within {wait_seconds:g}s",
        }
    return {"launched": True, "report": f"launch running: {package} {launch_file} via {result['backend']} backend"}


@node(
    name="ROS2Run",
    category=_CATEGORY,
    description="Start or stop a background `ros2 run <package> <executable> ...` process.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "run_id": Text(default="ros2_run"),
        "package": Text(default=""),
        "executable": Text(default=""),
        "arguments": Text(default=""),
        "expected_topic": Text(default=""),
        "wait_seconds": Float(default=0.0),
    },
    outputs={"running": Bool, "run_id": Text, "report": Text},
)
def ros2_run(ctx: dict) -> dict:
    run_id = str(ctx.get("run_id") or "ros2_run").strip() or "ros2_run"
    action = str(ctx.get("action") or "start").strip().lower()
    package = str(ctx.get("package") or "").strip()
    executable = str(ctx.get("executable") or "").strip()
    pattern = " ".join(part for part in ("ros2", "run", package, executable) if part)

    if action == "stop":
        result = rt.stop_ros2_managed(run_id, pattern=pattern or "ros2 run")
        if result.get("ok"):
            return {
                "running": False,
                "run_id": run_id,
                "report": f"stopped {result.get('stopped', 0)} ROS 2 run process(es)",
            }
        return {"running": False, "run_id": run_id, "report": _report(result, f"stop run {package} {executable}")}

    if not package or not executable:
        return {"running": False, "run_id": run_id, "report": "ros2 run FAILED: set package and executable"}
    try:
        extra_args = shlex.split(str(ctx.get("arguments") or ""))
    except ValueError as exc:
        return {"running": False, "run_id": run_id, "report": f"ros2 run FAILED: invalid arguments: {exc}"}

    result = rt.run_ros2_managed(run_id, ["run", package, executable, *extra_args])
    if not result.get("ok"):
        return {"running": False, "run_id": run_id, "report": _report(result, f"start run {package} {executable}")}

    expected_topic = str(ctx.get("expected_topic") or "").strip()
    wait_seconds = max(0.0, float(ctx.get("wait_seconds") or 0.0))
    if expected_topic and wait_seconds > 0:
        deadline = time.time() + wait_seconds
        while time.time() < deadline:
            check = rt.run_ros2(["topic", "list"], timeout=10)
            topics = {line.strip().split()[0] for line in check.get("stdout", "").splitlines() if line.strip()}
            if check.get("ok") and expected_topic in topics:
                return {
                    "running": True,
                    "run_id": run_id,
                    "report": (
                        f"ROS 2 run process running: {package} {executable}; "
                        f"{expected_topic} is discoverable via {result['backend']} backend"
                    ),
                }
            time.sleep(1)
        return {
            "running": True,
            "run_id": run_id,
            "report": (
                f"ROS 2 run process started: {package} {executable}, "
                f"but {expected_topic} was not discoverable within {wait_seconds:g}s"
            ),
        }

    return {
        "running": True,
        "run_id": run_id,
        "report": f"ROS 2 run process running: {package} {executable} via {result['backend']} backend",
    }


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
    name="ROS2PackageExecutables",
    category=_CATEGORY,
    description="List executables registered by a ROS 2 package (`ros2 pkg executables`).",
    inputs={
        "trigger": AnyPort,
        "package": Text(default="demo_nodes_cpp"),
        "timeout": Float(default=30.0),
    },
    outputs={"executables": List, "report": Text},
)
def ros2_package_executables(ctx: dict) -> dict:
    package = str(ctx.get("package") or "").strip()
    if not package:
        return {"executables": [], "report": "pkg executables FAILED: set package"}
    result = rt.run_ros2(
        ["pkg", "executables", package],
        timeout=max(1.0, float(ctx.get("timeout") or 30.0)),
    )
    executables = [line.strip() for line in result["stdout"].splitlines() if line.strip()] if result["ok"] else []
    return {"executables": executables, "report": _report(result, f"pkg executables {package}")}


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
