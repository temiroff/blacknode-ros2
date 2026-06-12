"""ROS 2 nodes for Blacknode.

Topic, service, node, and interface introspection plus publishing, backed by
a native ``ros2`` CLI or a Docker helper container (see ``ros2_runtime``).
Every node returns a structured report instead of raising, so workflows stay
usable on machines without ROS.

The ``trigger`` input is an optional pass-through: wire any upstream port
into it to sequence ROS actions (e.g. start a demo publisher before echoing).
"""
from __future__ import annotations

import shlex
import time
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Enum, Float, Int, List, Text, node

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
