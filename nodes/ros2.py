"""ROS 2 nodes for Blacknode.

Topic, service, node, and interface introspection plus publishing, backed by
a native ``ros2`` CLI or a Docker helper container (see ``ros2_runtime``).
Every node returns a structured report instead of raising, so workflows stay
usable on machines without ROS.

The ``trigger`` input is an optional pass-through: wire any upstream port
into it to sequence ROS actions (e.g. start a topic publisher before echoing).
"""
from __future__ import annotations

import base64
from concurrent.futures import ThreadPoolExecutor, as_completed
import html
import re
import shlex
import time
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text

from . import ros2_runtime as rt
from ._implementation import implementation_node as node

_CATEGORY = "ROS 2"


def _report(result: dict[str, Any], action: str) -> str:
    if result.get("ok"):
        return f"{action} OK via {result.get('backend', '?')} backend"
    return f"{action} FAILED: {result.get('error', 'unknown error')}"


def _managed_id(value: Any, fallback: str) -> str:
    return (
        re.sub(r"[^a-zA-Z0-9_-]+", "_", str(value or fallback).strip())
        .strip("_")[:64]
        or fallback
    )


@node(
    name="ROS2SystemCheck", component="diagnostics",
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
    name="ROS2TopicList", component="topics",
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
    name="ROS2TopicEcho", component="topics",
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
    name="ROS2TopicSubscriber", component="topics",
    category=_CATEGORY,
    description="Subscribe once, start continuously, or stop a named ROS 2 subscriber with structured live messages.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["once", "start", "stop"], default="start"),
        "node_name": Text(default="blacknode_subscriber"),
        "topic": Text(default="/chatter"),
        "msg_type": Text(default="std_msgs/msg/String"),
        "history": Int(default=10),
        "timeout": Float(default=10.0),
    },
    outputs={
        "running": Bool,
        "latest": Dict,
        "messages": List,
        "received": Int,
        "backend": Text,
        "report": Text,
    },
)
def ros2_topic_subscriber(ctx: dict) -> dict:
    action = str(ctx.get("action") or "start").strip().lower()
    node_name = str(ctx.get("node_name") or "blacknode_subscriber").strip().lstrip("/")
    topic = str(ctx.get("topic") or "/chatter").strip() or "/chatter"
    msg_type = str(ctx.get("msg_type") or "std_msgs/msg/String").strip() or "std_msgs/msg/String"
    backend = rt.detect_backend()["backend"]
    empty = {
        "running": False,
        "latest": {},
        "messages": [],
        "received": 0,
        "backend": backend,
    }
    if action not in {"once", "start", "stop"}:
        return {
            **empty,
            "report": f"topic subscriber FAILED: action must be once, start, or stop, got {action!r}",
        }
    if action == "stop":
        result = rt.stop_topic_subscriber(topic)
        messages = list(result.get("messages") or [])
        return {
            **empty,
            "latest": messages[-1] if messages else {},
            "messages": messages,
            "received": int(result.get("received") or len(messages)),
            "backend": result.get("backend", backend),
            "report": (
                f"stopped topic subscriber on {topic}"
                if result.get("ok")
                else _report(result, f"stop topic subscriber on {topic}")
            ),
        }
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", node_name):
        return {
            **empty,
            "report": (
                "topic subscriber FAILED: node_name must start with a letter or "
                "underscore and contain only letters, numbers, and underscores"
            ),
        }
    try:
        history = max(1, min(100, int(ctx.get("history") or 10)))
    except (TypeError, ValueError):
        history = 10
    try:
        timeout = max(0.1, float(ctx.get("timeout") or 10.0))
    except (TypeError, ValueError):
        timeout = 10.0
    if action == "once":
        result = rt.run_topic_subscriber_once(
            topic=topic,
            message_type=msg_type,
            node_name=node_name,
            timeout=timeout,
        )
        messages = list(result.get("messages") or [])
        return {
            "running": False,
            "latest": messages[-1] if messages else {},
            "messages": messages,
            "received": int(result.get("received") or len(messages)),
            "backend": result.get("backend", backend),
            "report": (
                f"received one message as /{node_name} from {topic} via {result.get('backend', backend)}"
                if result.get("ok")
                else _report(result, f"subscribe once to {topic}")
            ),
        }
    result = rt.start_topic_subscriber(
        topic=topic,
        message_type=msg_type,
        node_name=node_name,
        history=history,
    )
    if not result.get("ok"):
        return {
            **empty,
            "backend": result.get("backend", backend),
            "report": _report(result, f"start topic subscriber on {topic}"),
        }
    return {
        **empty,
        "running": True,
        "backend": result.get("backend", backend),
        "report": (
            f"topic subscriber running as /{node_name} on {topic} ({msg_type}) "
            f"via {result.get('backend', backend)}"
        ),
    }


def _ros2_message_type(topic: str, configured: str) -> tuple[str, str]:
    if configured:
        return configured, ""
    discovered = rt.run_ros2(["topic", "type", topic], timeout=15)
    message_type = next(
        (line.strip() for line in str(discovered.get("stdout") or "").splitlines() if line.strip()),
        "",
    )
    if discovered.get("ok") and message_type:
        return message_type, ""
    reason = str(
        discovered.get("error")
        or discovered.get("stderr")
        or f"no publisher advertises {topic}"
    )
    return "", f"could not discover the message type for {topic}: {reason}"


@node(
    name="ROS2", component="topics",
    category=_CATEGORY,
    description=(
        "Read one configured ROS 2 topic as a managed message stream. "
        "Connect a ComputeDevice to run on a paired device, or leave it empty "
        "to use the local Runtime."
    ),
    inputs={
        "trigger": AnyPort,
        "device": Dict,
        "action": Enum(["once", "start", "status", "stop"], default="status"),
        "topic": Text(default="/scan"),
        "message_type": Text(default=""),
        "node_name": Text(default="blacknode_ros2_topic"),
        "history": Int(default=10),
        "timeout": Float(default=10.0),
        "stale_after_seconds": Float(default=2.0),
    },
    outputs={
        "running": Bool,
        "message": Dict,
        "messages": List,
        "stream": Dict,
        "status": Dict,
        "received": Int,
        "backend": Text,
        "report": Text,
    },
    primary_inputs=["device", "action", "topic", "message_type"],
    primary_outputs=["stream", "status", "message"],
    live=True,
)
def ros2_topic(ctx: dict) -> dict:
    action = str(ctx.get("action") or "status").strip().lower()
    topic = str(ctx.get("topic") or "/scan").strip() or "/scan"
    configured_type = str(ctx.get("message_type") or "").strip()
    node_name = str(ctx.get("node_name") or "blacknode_ros2_topic").strip().lstrip("/")
    try:
        history = max(1, min(100, int(ctx.get("history") or 10)))
    except (TypeError, ValueError):
        history = 10
    try:
        timeout = max(0.1, float(ctx.get("timeout") or 10.0))
    except (TypeError, ValueError):
        timeout = 10.0
    try:
        stale_after_seconds = max(0.05, float(ctx.get("stale_after_seconds") or 2.0))
    except (TypeError, ValueError):
        stale_after_seconds = 2.0
    device = ctx.get("device") if isinstance(ctx.get("device"), dict) else {}
    device_id = str(device.get("device_id") or "").strip()
    backend_hint = f"remote:{device_id}" if device_id else rt.detect_backend()["backend"]

    if action not in {"once", "start", "status", "stop"}:
        status = {
            "running": False,
            "backend": backend_hint,
            "topic": topic,
            "message_type": configured_type,
            "service_id": f"topic-subscriber:{topic}",
            "stale_after_seconds": stale_after_seconds,
            "error": f"action must be once, start, status, or stop, got {action!r}",
        }
        return rt.ros2_topic_outputs(status, report=f"ROS2 FAILED: {status['error']}")

    if device_id:
        remote_action = ctx.get("__remote_ros2_action__")
        if not callable(remote_action):
            status = {
                "running": False,
                "backend": "none",
                "topic": topic,
                "message_type": configured_type,
                "service_id": f"device:{device_id}:topic-subscriber:{topic}",
                "stale_after_seconds": stale_after_seconds,
                "state": "unavailable",
                "error": (
                    "paired-device ROS2 streaming is available through the "
                    "Blacknode editor Runtime"
                ),
            }
            return rt.ros2_topic_outputs(status, report=f"ROS2 FAILED: {status['error']}")
        try:
            result = remote_action({
                "node_id": str(ctx.get("__node_id__") or ""),
                "device_id": device_id,
                "action": action,
                "topic": topic,
                "message_type": configured_type,
                "node_name": node_name,
                "history": history,
                "timeout": timeout,
                "stale_after_seconds": stale_after_seconds,
            })
        except Exception as exc:  # editor service boundary returns structured state
            status = {
                "running": False,
                "backend": "none",
                "topic": topic,
                "message_type": configured_type,
                "service_id": f"device:{device_id}:topic-subscriber:{topic}",
                "stale_after_seconds": stale_after_seconds,
                "state": "unavailable",
                "error": str(exc),
            }
            return rt.ros2_topic_outputs(status, report=f"ROS2 FAILED: {status['error']}")
        outputs = result.get("outputs") if isinstance(result, dict) else None
        if isinstance(outputs, dict):
            return outputs
        status = {
            "running": False,
            "backend": "none",
            "topic": topic,
            "message_type": configured_type,
            "service_id": f"device:{device_id}:topic-subscriber:{topic}",
            "stale_after_seconds": stale_after_seconds,
            "state": "error",
            "error": "paired Runtime returned an invalid ROS2 stream response",
        }
        return rt.ros2_topic_outputs(status, report=f"ROS2 FAILED: {status['error']}")

    if action == "status":
        status = rt.topic_subscriber_status(topic)
        status["stale_after_seconds"] = stale_after_seconds
        age_seconds = status.get("age_seconds")
        status["source_fresh"] = bool(
            status.get("received")
            and isinstance(age_seconds, (int, float))
            and float(age_seconds) <= stale_after_seconds
        )
        if status.get("backend") == "none":
            report = "ROS2 unavailable: Install ROS 2 on the Runtime device"
        else:
            report = (
                f"ROS2 status: {topic} is {('running' if status.get('running') else 'stopped')}; "
                f"received {int(status.get('received') or 0)} message(s)"
            )
        return rt.ros2_topic_outputs(status, report=report)

    if action == "stop":
        stopped = rt.stop_topic_subscriber(topic)
        stopped.setdefault("topic", topic)
        stopped.setdefault("message_type", configured_type)
        stopped["state"] = "stopped"
        stopped["stale_after_seconds"] = stale_after_seconds
        return rt.ros2_topic_outputs(stopped, report=f"ROS2 stopped: {topic}")

    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", node_name):
        status = {
            "running": False,
            "backend": rt.detect_backend()["backend"],
            "topic": topic,
            "message_type": configured_type,
            "service_id": f"topic-subscriber:{topic}",
            "stale_after_seconds": stale_after_seconds,
            "error": (
                "node_name must start with a letter or underscore and contain "
                "only letters, numbers, and underscores"
            ),
        }
        return rt.ros2_topic_outputs(status, report=f"ROS2 FAILED: {status['error']}")

    message_type, type_error = _ros2_message_type(topic, configured_type)
    if type_error:
        status = {
            "running": False,
            "backend": rt.detect_backend()["backend"],
            "topic": topic,
            "message_type": "",
            "service_id": f"topic-subscriber:{topic}",
            "stale_after_seconds": stale_after_seconds,
            "error": type_error,
        }
        return rt.ros2_topic_outputs(status, report=f"ROS2 FAILED: {type_error}")

    if action == "once":
        result = rt.run_topic_subscriber_once(
            topic=topic,
            message_type=message_type,
            node_name=node_name,
            timeout=timeout,
            public_node_type="ROS2",
            stale_after_seconds=stale_after_seconds,
        )
        report = (
            f"ROS2 received one {message_type} message from {topic}"
            if result.get("ok")
            else f"ROS2 FAILED: {result.get('error') or 'no message received'}"
        )
        return rt.ros2_topic_outputs(result, report=report)

    started = rt.start_topic_subscriber(
        topic=topic,
        message_type=message_type,
        node_name=node_name,
        history=history,
        public_node_type="ROS2",
        stale_after_seconds=stale_after_seconds,
    )
    if not started.get("ok"):
        failed = {
            **started,
            "running": False,
            "topic": topic,
            "message_type": message_type,
            "service_id": f"topic-subscriber:{topic}",
            "stale_after_seconds": stale_after_seconds,
        }
        return rt.ros2_topic_outputs(
            failed,
            report=f"ROS2 FAILED: {started.get('error') or 'could not start subscription'}",
        )
    status = rt.topic_subscriber_status(topic)
    return rt.ros2_topic_outputs(
        status,
        report=f"ROS2 streaming {message_type} from {topic} via {started.get('backend', '?')}",
    )


@node(
    name="ROS2TopicPublisher", component="topics",
    category=_CATEGORY,
    description="Publish once, start continuously, or stop a publisher for any ROS 2 topic and message type.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["once", "start", "stop"], default="start"),
        "node_name": Text(default=""),
        "topic": Text(default="/chatter"),
        "msg_type": Text(default="std_msgs/msg/String"),
        "payload": Text(default="data: hello from Blacknode"),
        "count": Int(default=1),
        "rate_hz": Float(default=2.0),
    },
    outputs={"running": Bool, "backend": Text, "report": Text},
)
def ros2_topic_publisher(ctx: dict) -> dict:
    return _run_topic_publisher(ctx)


def _run_topic_publisher(ctx: dict) -> dict:
    action = str(ctx.get("action") or "start").strip().lower()
    node_name = str(ctx.get("node_name") or "").strip().lstrip("/")
    topic = str(ctx.get("topic") or "/chatter").strip() or "/chatter"
    backend = rt.detect_backend()["backend"]
    if action not in {"once", "start", "stop"}:
        return {
            "running": False,
            "backend": backend,
            "report": f"topic publisher FAILED: action must be once, start, or stop, got {action!r}",
        }
    key = f"topic-publisher:{topic}"
    if action == "stop":
        pattern = "" if backend == "native" else f"ros2 topic pub .* {topic} "
        result = rt.stop_ros2_managed(key, pattern=pattern)
        if result["ok"]:
            return {
                "running": False,
                "backend": result["backend"],
                "report": f"stopped topic publisher on {topic}",
            }
        return {
            "running": False,
            "backend": result["backend"],
            "report": _report(result, f"stop topic publisher on {topic}"),
        }
    if node_name and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", node_name):
        return {
            "running": False,
            "backend": backend,
            "report": (
                "topic publisher FAILED: node_name must start with a letter or "
                "underscore and contain only letters, numbers, and underscores"
            ),
        }
    msg_type = str(ctx.get("msg_type") or "std_msgs/msg/String").strip() or "std_msgs/msg/String"
    payload = str(ctx.get("payload") or "data: hello from Blacknode").strip()
    if action == "once":
        try:
            count = max(1, int(ctx.get("count") or 1))
        except (TypeError, ValueError):
            count = 1
        args = ["topic", "pub"]
        args.extend(["--once"] if count == 1 else ["--times", str(count)])
        args.extend(["--wait-matching-subscriptions", "0"])
        if node_name:
            args.extend(["--node-name", node_name])
        args.extend([topic, msg_type, payload])
        result = rt.run_ros2(args, timeout=30 + count)
        return {
            "running": False,
            "backend": result.get("backend", backend),
            "report": _report(result, f"publish {count}x to {topic}"),
        }
    try:
        rate_hz = float(ctx.get("rate_hz", 2.0))
    except (TypeError, ValueError):
        rate_hz = 0.0
    if rate_hz <= 0:
        return {
            "running": False,
            "backend": backend,
            "report": "topic publisher FAILED: rate_hz must be greater than 0",
        }
    # Docker exec does not return the child PID, and matching the complete
    # command is unreliable when structured payload punctuation is interpreted
    # as a regular expression. A managed publisher is topic-scoped, so replace
    # every older publisher for this exact topic before starting the new one.
    replace_pattern = "" if backend == "native" else f"ros2 topic pub .* {topic} "
    rt.stop_ros2_managed(key, pattern=replace_pattern)
    args = ["topic", "pub", "-r", str(rate_hz)]
    if node_name:
        args.extend(["--node-name", node_name])
    args.extend([topic, msg_type, payload])
    result = rt.run_ros2_managed(key, args)
    if not result["ok"]:
        return {
            "running": False,
            "backend": result["backend"],
            "report": _report(result, f"start topic publisher on {topic}"),
        }
    # Wait until DDS discovery sees the topic, so downstream nodes wired to
    # this report can echo immediately instead of racing discovery.
    deadline = time.time() + 15
    while time.time() < deadline:
        check = rt.run_ros2(["topic", "list"], timeout=10)
        if check["ok"] and topic in check["stdout"].split():
            return {
                "running": True,
                "backend": result["backend"],
                "report": (
                    f"topic publisher running{f' as /{node_name}' if node_name else ''} "
                    f"on {topic} ({msg_type}) at {rate_hz:g} Hz via {result['backend']}"
                ),
            }
        time.sleep(1)
    return {
        "running": True,
        "backend": result["backend"],
        "report": f"topic publisher started on {topic} but the topic is not discoverable yet",
    }


_MOTION_TOPIC_TOKENS = {
    "command",
    "commands",
    "cmd_vel",
    "joint_command",
    "joint_commands",
    "robot_control",
    "servo_command",
    "servo_commands",
    "trajectory",
}


def _motion_destination(topic: str) -> bool:
    segments = {
        segment
        for segment in str(topic or "").strip().lower().split("/")
        if segment
    }
    return bool(segments & _MOTION_TOPIC_TOKENS)


@node(
    name="ROS2TopicRelay", component="topics",
    category=_CATEGORY,
    description=(
        "Continuously subscribe to one ROS 2 data topic and republish the same "
        "message type on another topic. Motion command topics are rejected; "
        "use a safety-gated controller for robot motion."
    ),
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "run_id": Text(default="topic_relay"),
        "source_topic": Text(default="/source"),
        "destination_topic": Text(default="/destination"),
        "msg_type": Text(default="std_msgs/msg/String"),
        "qos": Enum(["sensor_data", "reliable"], default="sensor_data"),
        "queue_depth": Int(default=10),
    },
    outputs={
        "running": Bool,
        "backend": Text,
        "source_topic": Text,
        "destination_topic": Text,
        "report": Text,
    },
)
def ros2_topic_relay(ctx: dict) -> dict:
    action = str(ctx.get("action") or "start").strip().lower()
    run_id = re.sub(
        r"[^a-zA-Z0-9_-]+",
        "_",
        str(ctx.get("run_id") or "topic_relay").strip(),
    ).strip("_") or "topic_relay"
    source_topic = str(ctx.get("source_topic") or "/source").strip() or "/source"
    destination_topic = (
        str(ctx.get("destination_topic") or "/destination").strip()
        or "/destination"
    )
    message_type = (
        str(ctx.get("msg_type") or "std_msgs/msg/String").strip()
        or "std_msgs/msg/String"
    )
    backend = rt.detect_backend()["backend"]
    base = {
        "backend": backend,
        "source_topic": source_topic,
        "destination_topic": destination_topic,
    }
    if action == "stop":
        result = rt.stop_topic_relay(run_id)
        return {
            **base,
            "running": False,
            "backend": result.get("backend", backend),
            "report": (
                f"stopped topic relay {run_id}"
                if result.get("ok")
                else _report(result, f"stop topic relay {run_id}")
            ),
        }
    if action != "start":
        return {
            **base,
            "running": False,
            "report": f"topic relay FAILED: action must be start or stop, got {action!r}",
        }
    if source_topic == destination_topic:
        return {
            **base,
            "running": False,
            "report": "topic relay FAILED: source and destination topics must be different",
        }
    if _motion_destination(destination_topic):
        return {
            **base,
            "running": False,
            "report": (
                f"topic relay BLOCKED: {destination_topic} looks like a motion "
                "command topic. Use a safety-gated Blacknode controller so "
                "arming, freshness, calibration, and limits remain enforced."
            ),
        }
    qos = str(ctx.get("qos") or "sensor_data").strip().lower()
    if qos not in {"sensor_data", "reliable"}:
        qos = "sensor_data"
    try:
        queue_depth = max(1, int(ctx.get("queue_depth") or 10))
    except (TypeError, ValueError):
        queue_depth = 10
    result = rt.start_topic_relay(
        run_id=run_id,
        source_topic=source_topic,
        destination_topic=destination_topic,
        message_type=message_type,
        qos=qos,
        queue_depth=queue_depth,
    )
    return {
        **base,
        "running": bool(result.get("ok")),
        "backend": result.get("backend", backend),
        "report": (
            f"relaying {source_topic} -> {destination_topic} "
            f"({message_type}, {qos}) via {result.get('backend', backend)}"
            if result.get("ok")
            else _report(result, f"relay {source_topic} -> {destination_topic}")
        ),
    }


@node(
    name="ROS2WorkspaceBuild", component="processes",
    category=_CATEGORY,
    description="Build a local colcon workspace for ROS2Run and ROS2Launch.",
    inputs={
        "trigger": AnyPort,
        "workspace_path": Text(default=""),
        "packages_select": Text(default=""),
        "timeout": Float(default=300.0),
    },
    outputs={
        "built": Bool,
        "backend": Text,
        "workspace_path": Text,
        "setup_path": Text,
        "logs": List,
        "report": Text,
    },
    primary_inputs=["trigger", "workspace_path", "packages_select"],
    primary_outputs=["built", "logs", "report"],
)
def ros2_workspace_build(ctx: dict) -> dict:
    workspace_path = str(ctx.get("workspace_path") or "").strip()
    try:
        packages = shlex.split(str(ctx.get("packages_select") or ""))
    except ValueError as exc:
        backend = rt.detect_backend()["backend"]
        return {
            "built": False,
            "backend": backend,
            "workspace_path": workspace_path,
            "setup_path": "",
            "logs": [],
            "report": f"ROS 2 workspace build FAILED: invalid packages_select: {exc}",
        }
    try:
        timeout = max(30.0, float(ctx.get("timeout") or 300.0))
    except (TypeError, ValueError):
        timeout = 300.0
    result = rt.build_ros2_workspace(
        workspace_path,
        packages_select=packages,
        timeout=timeout,
    )
    logs = [
        line
        for line in "\n".join(
            part for part in (result.get("stdout", ""), result.get("stderr", "")) if part
        ).splitlines()[-100:]
        if line.strip()
    ]
    if not result.get("ok"):
        return {
            "built": False,
            "backend": result.get("backend", "none"),
            "workspace_path": result.get("workspace_path", workspace_path),
            "setup_path": result.get("setup_path", ""),
            "logs": logs,
            "report": _report(result, "ROS 2 workspace build"),
        }
    selected = f" ({', '.join(packages)})" if packages else ""
    return {
        "built": True,
        "backend": result.get("backend", "none"),
        "workspace_path": result.get("workspace_path", workspace_path),
        "setup_path": result.get("setup_path", ""),
        "logs": logs,
        "report": (
            f"ROS 2 workspace built{selected} at {result.get('workspace_path', workspace_path)} "
            f"via {result.get('backend', 'none')} backend"
        ),
    }


@node(
    name="ROS2PythonNode", component="processes",
    category=_CATEGORY,
    description="Start or stop a standalone Python rclpy script from a file or inline code.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "run_id": Text(default="ros2_python_node"),
        "source_mode": Enum(["file", "inline"], default="file"),
        "script_path": Text(default=""),
        "code": Text(default=""),
        "arguments": Text(default=""),
    },
    outputs={
        "running": Bool,
        "run_id": Text,
        "backend": Text,
        "script": Text,
        "logs": List,
        "report": Text,
    },
    primary_inputs=["trigger", "action", "run_id", "source_mode", "script_path"],
    primary_outputs=["running", "logs", "report"],
)
def ros2_python_node(ctx: dict) -> dict:
    action = str(ctx.get("action") or "start").strip().lower()
    run_id = _managed_id(ctx.get("run_id"), "ros2_python_node")
    source_mode = str(ctx.get("source_mode") or "file").strip().lower()
    script_path = str(ctx.get("script_path") or "").strip()
    code = str(ctx.get("code") or "")
    backend = rt.detect_backend()["backend"]
    script = script_path if source_mode == "file" else "inline code"
    base = {
        "running": False,
        "run_id": run_id,
        "backend": backend,
        "script": script,
        "logs": [],
    }
    if action == "stop":
        result = rt.stop_ros2_python_node(run_id)
        return {
            **base,
            "backend": result.get("backend", backend),
            "report": (
                f"stopped ROS 2 Python node {run_id}"
                if result.get("ok")
                else _report(result, f"stop ROS 2 Python node {run_id}")
            ),
        }
    if action != "start":
        return {
            **base,
            "report": f"ROS 2 Python node FAILED: action must be start or stop, got {action!r}",
        }
    if source_mode not in {"file", "inline"}:
        return {
            **base,
            "report": f"ROS 2 Python node FAILED: source_mode must be file or inline, got {source_mode!r}",
        }
    if source_mode == "file" and not script_path:
        return {**base, "report": "ROS 2 Python node FAILED: set script_path for file mode"}
    if source_mode == "inline" and not code.strip():
        return {**base, "report": "ROS 2 Python node FAILED: enter code for inline mode"}
    try:
        arguments = shlex.split(str(ctx.get("arguments") or ""))
    except ValueError as exc:
        return {**base, "report": f"ROS 2 Python node FAILED: invalid arguments: {exc}"}
    result = rt.start_ros2_python_node(
        run_id=run_id,
        source_mode=source_mode,
        script_path=script_path,
        code=code,
        arguments=arguments,
    )
    if not result.get("ok"):
        return {
            **base,
            "backend": result.get("backend", backend),
            "report": _report(result, f"start ROS 2 Python node {run_id}"),
        }
    return {
        **base,
        "running": True,
        "backend": result.get("backend", backend),
        "script": result.get("script", script),
        "report": (
            f"ROS 2 Python node {run_id} running from {result.get('script', script)} "
            f"via {result.get('backend', backend)}"
        ),
    }


@node(
    name="ROS2Launch", component="processes",
    category=_CATEGORY,
    description="Start or stop a background `ros2 launch ...` process.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "run_id": Text(default="ros2_launch"),
        "package": Text(default=""),
        "launch_file": Text(default=""),
        "workspace_path": Text(default=""),
        "arguments": Text(default=""),
        "expected_topic": Text(default=""),
        "wait_seconds": Float(default=0.0),
        "stop_pattern": Text(default=""),
    },
    outputs={"launched": Bool, "run_id": Text, "report": Text},
)
def ros2_launch(ctx: dict) -> dict:
    action = str(ctx.get("action") or "start")
    run_id = _managed_id(ctx.get("run_id"), "ros2_launch")
    package = str(ctx.get("package") or "").strip()
    launch_file = str(ctx.get("launch_file") or "").strip()
    workspace_path = str(ctx.get("workspace_path") or "").strip()

    if action == "stop":
        pattern = str(ctx.get("stop_pattern") or "").strip() or f"ros2 launch {package}".strip() or "ros2 launch"
        result = rt.stop_ros2_managed(run_id, pattern=pattern)
        if result["ok"]:
            return {
                "launched": False,
                "run_id": run_id,
                "report": (
                    f"stopped {result.get('stopped', 0)} background launch "
                    "process(es)"
                ),
            }
        return {
            "launched": False,
            "run_id": run_id,
            "report": _report(result, f"stop launch {package}"),
        }

    if not package or not launch_file:
        return {
            "launched": False,
            "run_id": run_id,
            "report": "ros2 launch FAILED: set package and launch_file",
        }
    try:
        extra_args = shlex.split(str(ctx.get("arguments") or ""))
    except ValueError as exc:
        return {
            "launched": False,
            "run_id": run_id,
            "report": f"ros2 launch FAILED: invalid arguments: {exc}",
        }

    managed_kwargs = {"workspace_path": workspace_path} if workspace_path else {}
    result = rt.run_ros2_managed(
        run_id,
        ["launch", package, launch_file, *extra_args],
        **managed_kwargs,
    )
    if not result["ok"]:
        return {
            "launched": False,
            "run_id": run_id,
            "report": _report(result, f"start launch {package} {launch_file}"),
        }

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
                    "run_id": run_id,
                    "report": (
                        f"launch running: {package} {launch_file}; "
                        f"{expected_topic} is discoverable via {result['backend']} backend"
                    ),
                }
            time.sleep(1)
        return {
            "launched": True,
            "run_id": run_id,
            "report": f"launch started: {package} {launch_file}, but {expected_topic} was not discoverable within {wait_seconds:g}s",
        }
    return {
        "launched": True,
        "run_id": run_id,
        "report": (
            f"launch running: {package} {launch_file} via "
            f"{result['backend']} backend"
        ),
    }


@node(
    name="ROS2Run", component="processes",
    category=_CATEGORY,
    description="Start or stop a background `ros2 run <package> <executable> ...` process.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["start", "stop"], default="start"),
        "run_id": Text(default="ros2_run"),
        "package": Text(default=""),
        "executable": Text(default=""),
        "workspace_path": Text(default=""),
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
    workspace_path = str(ctx.get("workspace_path") or "").strip()
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

    managed_kwargs = {"workspace_path": workspace_path} if workspace_path else {}
    result = rt.run_ros2_managed(
        run_id,
        ["run", package, executable, *extra_args],
        **managed_kwargs,
    )
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
    name="ROS2NodeList", component="diagnostics",
    category=_CATEGORY,
    description="List running ROS 2 nodes.",
    inputs={"trigger": AnyPort},
    outputs={"nodes": List, "report": Text},
)
def ros2_node_list(ctx: dict) -> dict:
    result = rt.run_ros2(["node", "list"], timeout=30)
    nodes = [line.strip() for line in result["stdout"].splitlines() if line.strip()] if result["ok"] else []
    return {"nodes": nodes, "report": _report(result, "node list")}


_SYSTEM_TOPICS = {"/parameter_events", "/rosout"}


def _typed_names(output: str) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for raw_line in str(output or "").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        match = re.match(r"^(.*?)\s+\[(.*)]$", line)
        if match:
            name = match.group(1).strip()
            message_types = [
                item.strip()
                for item in match.group(2).split(",")
                if item.strip()
            ]
        else:
            name = line
            message_types = []
        entries.append({"name": name, "types": message_types})
    return entries


def _in_namespace(name: str, namespace: str) -> bool:
    selected = str(namespace or "").strip()
    if not selected or selected == "/":
        return True
    if not selected.startswith("/"):
        selected = f"/{selected}"
    selected = selected.rstrip("/")
    return name == selected or name.startswith(f"{selected}/")


def _qualified_ros_node(name: str, namespace: str) -> str:
    node_name = str(name or "").strip()
    node_namespace = str(namespace or "/").strip() or "/"
    if node_name.startswith("/"):
        return re.sub(r"/+", "/", node_name)
    if node_namespace == "/":
        return f"/{node_name}" if node_name else "/"
    return re.sub(r"/+", "/", f"/{node_namespace.strip('/')}/{node_name}")


def _parse_topic_endpoint_details(output: str) -> dict[str, Any]:
    text = str(output or "")
    publisher_match = re.search(r"^Publisher count:\s*(\d+)", text, re.MULTILINE)
    subscriber_match = re.search(r"^Subscription count:\s*(\d+)", text, re.MULTILINE)
    publishers: list[dict[str, Any]] = []
    subscribers: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    qos: dict[str, str] | None = None

    def finish() -> None:
        nonlocal current, qos
        if not current:
            return
        current["node"] = _qualified_ros_node(
            str(current.pop("node_name", "")),
            str(current.pop("node_namespace", "/")),
        )
        endpoint_type = str(current.pop("endpoint_type", "")).lower()
        if qos:
            current["qos"] = qos
        if endpoint_type == "publisher":
            publishers.append(current)
        elif endpoint_type in {"subscription", "subscriber"}:
            subscribers.append(current)
        current = None
        qos = None

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line.startswith("Node name:"):
            finish()
            current = {"node_name": line.partition(":")[2].strip()}
            continue
        if current is None:
            continue
        if line.startswith("QoS profile:"):
            qos = {}
            continue
        if ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        normalized = key.lower().replace(" ", "_").replace("(", "").replace(")", "")
        if qos is not None and normalized in {
            "reliability", "history_depth", "durability", "lifespan",
            "deadline", "liveliness", "liveliness_lease_duration",
        }:
            qos[normalized] = value
        elif normalized in {"node_namespace", "topic_type", "endpoint_type", "gid"}:
            current[normalized] = value
    finish()
    return {
        "publisher_count": int(publisher_match.group(1)) if publisher_match else len(publishers),
        "subscription_count": int(subscriber_match.group(1)) if subscriber_match else len(subscribers),
        "publishers": publishers,
        "subscribers": subscribers,
    }


@node(
    name="ROS2GraphExplorer", component="diagnostics",
    category=_CATEGORY,
    description=(
        "Capture a structured, read-only ROS 2 topology: nodes, typed topics, "
        "services, publisher/subscriber endpoints, and QoS summaries."
    ),
    inputs={
        "trigger": AnyPort,
        "namespace": Text(default="/"),
        "include_system": Bool(default=False),
        "include_endpoints": Bool(default=True),
        "max_topics": Int(default=40),
        "timeout": Float(default=8.0),
    },
    outputs={
        "available": Bool,
        "backend": Text,
        "graph": Dict,
        "nodes": List,
        "topics": List,
        "services": List,
        "report": Text,
    },
)
def ros2_graph_explorer(ctx: dict) -> dict:
    backend = rt.detect_backend()["backend"]
    empty_graph = {
        "schema_version": 1,
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "backend": backend,
        "namespace": str(ctx.get("namespace") or "/"),
        "nodes": [],
        "topics": [],
        "services": [],
        "errors": [],
        "truncated": False,
    }
    if backend == "none":
        detail = rt.detect_backend().get("detail", "ROS 2 is unavailable")
        return {
            "available": False,
            "backend": backend,
            "graph": {**empty_graph, "errors": [detail]},
            "nodes": [],
            "topics": [],
            "services": [],
            "report": f"ROS 2 graph unavailable: {detail}",
        }

    namespace = str(ctx.get("namespace") or "/").strip() or "/"
    include_system = bool(ctx.get("include_system", False))
    include_endpoints = bool(ctx.get("include_endpoints", True))
    try:
        max_topics = min(200, max(1, int(ctx.get("max_topics") or 40)))
    except (TypeError, ValueError):
        max_topics = 40
    try:
        timeout = min(30.0, max(1.0, float(ctx.get("timeout") or 8.0)))
    except (TypeError, ValueError):
        timeout = 8.0

    node_result = rt.run_ros2(["node", "list"], timeout=timeout)
    topic_result = rt.run_ros2(["topic", "list", "-t"], timeout=timeout)
    service_result = rt.run_ros2(["service", "list", "-t"], timeout=timeout)
    results = [node_result, topic_result, service_result]
    errors = [
        str(result.get("error") or result.get("stderr") or "ROS graph query failed")
        for result in results
        if not result.get("ok")
    ]
    nodes = sorted({
        line.strip()
        for line in node_result.get("stdout", "").splitlines()
        if line.strip() and _in_namespace(line.strip(), namespace)
    }) if node_result.get("ok") else []
    topics = [
        entry
        for entry in _typed_names(topic_result.get("stdout", ""))
        if _in_namespace(entry["name"], namespace)
        and (include_system or entry["name"] not in _SYSTEM_TOPICS)
    ] if topic_result.get("ok") else []
    services = [
        entry
        for entry in _typed_names(service_result.get("stdout", ""))
        if _in_namespace(entry["name"], namespace)
    ] if service_result.get("ok") else []
    topics.sort(key=lambda item: item["name"])
    services.sort(key=lambda item: item["name"])
    truncated = len(topics) > max_topics
    topics = topics[:max_topics]

    if include_endpoints and topics:
        workers = min(8, len(topics))
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures = {
                executor.submit(
                    rt.run_ros2,
                    ["topic", "info", "-v", topic["name"]],
                    timeout,
                ): topic
                for topic in topics
            }
            for future in as_completed(futures):
                topic = futures[future]
                try:
                    result = future.result()
                except Exception as exc:  # noqa: BLE001 - preserve partial graph
                    topic.update({
                        "publisher_count": 0,
                        "subscription_count": 0,
                        "publishers": [],
                        "subscribers": [],
                    })
                    errors.append(f"{topic['name']}: {type(exc).__name__}: {exc}")
                    continue
                if result.get("ok"):
                    topic.update(_parse_topic_endpoint_details(result.get("stdout", "")))
                else:
                    topic.update({
                        "publisher_count": 0,
                        "subscription_count": 0,
                        "publishers": [],
                        "subscribers": [],
                    })
                    errors.append(
                        f"{topic['name']}: {result.get('error') or 'endpoint inspection failed'}"
                    )
    else:
        for topic in topics:
            topic.update({
                "publisher_count": 0,
                "subscription_count": 0,
                "publishers": [],
                "subscribers": [],
            })

    endpoint_nodes = {
        endpoint.get("node", "")
        for topic in topics
        for key in ("publishers", "subscribers")
        for endpoint in topic.get(key, [])
        if endpoint.get("node")
    }
    nodes = sorted(set(nodes) | endpoint_nodes)
    graph = {
        **empty_graph,
        "backend": next((result.get("backend") for result in results if result.get("backend")), backend),
        "namespace": namespace,
        "nodes": nodes,
        "topics": topics,
        "services": services,
        "errors": errors[:50],
        "truncated": truncated,
    }
    available = bool(any(result.get("ok") for result in results))
    report = (
        f"ROS 2 topology captured via {graph['backend']}: {len(nodes)} nodes, "
        f"{len(topics)} topics, {len(services)} services"
    )
    if truncated:
        report += f" (limited to {max_topics} topics)"
    if errors:
        report += f"; {len(errors)} query warning(s)"
    return {
        "available": available,
        "backend": graph["backend"],
        "graph": graph,
        "nodes": nodes,
        "topics": topics,
        "services": services,
        "report": report,
    }


@node(
    name="ROS2ServiceList", component="services",
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
    name="ROS2InterfaceShow", component="diagnostics",
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
    name="ROS2PackageExecutables", component="processes",
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


def _svg_text(value: Any, limit: int = 90) -> str:
    text = " ".join(str(value or "").split())
    if len(text) > limit:
        text = text[:limit - 3] + "..."
    return html.escape(text)


def _svg_data(svg: str) -> str:
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


@node(
    name="ROS2VisualDashboard", component="diagnostics",
    category=_CATEGORY,
    description="Render ROS 2 roundtrip results as a visual pass/fail dashboard.",
    hidden=True,
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
