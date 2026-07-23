"""Transport preflight for live ROS 2 robots.

``ROS2Status`` checks a robot connection over the best available transport:
native ``rclpy`` when importable, otherwise rosbridge (starting the local
rosbridge server when asked). The joint-space motion nodes that build on these
transports live in the ``blacknode-controllers`` joint-control ROS 2 adapter;
camera streaming lives in the ``blacknode-perception`` camera ROS 2 adapter.

Every node returns a structured report instead of raising, so workflows stay
usable on machines without ROS.
"""
from __future__ import annotations

import os
import shutil
import socket
import subprocess
import sys
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Int, List, Text

from . import ros2_native_runtime as nr
from . import rosbridge_runtime as rb
from . import rosbridge_service
from ._implementation import implementation_node as node

_CATEGORY = "ROS 2"


def _resolve_transport(ctx: dict) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    if requested in {"native", "rosbridge"}:
        return requested
    native_ok, _ = nr.available()
    return "native" if native_ok else "rosbridge"


def _transport_report(ctx: dict, resolved: str) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    suffix = " (auto-selected)" if requested == "auto" else ""
    return f"transport: {resolved}{suffix}"


def _loopback_host(host: str) -> bool:
    value = str(host or "").strip().lower().strip("[]")
    return value in {"localhost", "127.0.0.1", "::1"} or value.startswith("127.")


def _tcp_port_state(host: str, port: int, timeout: float = 0.5) -> str:
    try:
        with socket.create_connection((host, int(port)), timeout=max(0.1, timeout)):
            return "open"
    except ConnectionRefusedError:
        return "closed"
    except TimeoutError:
        return "unreachable"
    except OSError as exc:
        return f"unreachable ({exc})"


def _ros_distro_hint() -> str:
    distro = os.environ.get("ROS_DISTRO", "").strip()
    if distro:
        return distro
    ros2 = shutil.which("ros2") or ""
    marker = "/opt/ros/"
    if marker in ros2:
        candidate = ros2.split(marker, 1)[1].split("/", 1)[0].strip()
        if candidate:
            return candidate
    return "jazzy"


def _rosbridge_connection_diagnostics(host: str, port: int) -> list[str]:
    lines = [f"tcp port: {_tcp_port_state(host, port)} at {host}:{port}"]

    if not _loopback_host(host):
        lines.append("FIX: start rosbridge_server on the robot and set host to that machine's IP/name")
        return lines

    ros2 = shutil.which("ros2")
    if not ros2:
        lines.append("local ros2 CLI: not found")
        lines.append("FIX: install ROS 2 or run rosbridge on the robot and set host to that machine's IP/name")
        return lines

    lines.append(f"local ros2 CLI: {ros2}")
    try:
        check = subprocess.run(
            ["ros2", "pkg", "prefix", "rosbridge_server"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception as exc:  # noqa: BLE001
        lines.append(f"local rosbridge_server: check failed ({type(exc).__name__}: {exc})")
        lines.append("FIX: verify ROS 2 is sourced, then start rosbridge_server on port 9090")
        return lines

    if check.returncode == 0:
        lines.append("local rosbridge_server: installed")
        lines.append(f"FIX: start it with: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={port}")
        return lines

    distro = _ros_distro_hint()
    lines.append("local rosbridge_server: not found")
    lines.append(f"FIX: sudo apt install ros-{distro}-rosbridge-server")
    lines.append(f"then: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={port}")
    return lines


@node(
    name="ROS2RosbridgeStatus", component="rosbridge",
    category=_CATEGORY,
    hidden=True,
    description="Preflight a rosbridge robot connection: checks roslibpy, the WebSocket, and (optionally) a config topic, with the exact fix for anything missing.",
    inputs={
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "config_topic": Text(default=""),
        "timeout": Float(default=10.0),
    },
    outputs={"connected": Bool, "ready": Bool, "config": Dict, "report": Text},
)
def ros2_rosbridge_status(ctx: dict) -> dict:
    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)
    config_topic = str(ctx.get("config_topic") or "").strip()
    timeout = float(ctx.get("timeout") or 10.0)

    lines = [f"python:    {sys.executable} ({sys.version.split()[0]})"]
    ok, err = rb.available()
    if not ok:
        lines.append(f"roslibpy:  MISSING")
        lines.append(f'           FIX: "{sys.executable}" -m pip install roslibpy')
        lines.append("           (or press 'Install prerequisites' in the Packages tab)")
        lines.append("=> NOT READY")
        return {"connected": False, "ready": False, "config": {}, "report": "\n".join(lines)}
    lines.append(f"roslibpy:  OK ({rb.roslibpy_version()})")

    try:
        rb.get_connection(host, port, timeout)
    except Exception as exc:
        lines.append(f"rosbridge: UNREACHABLE at ws://{host}:{port} ({exc})")
        for line in _rosbridge_connection_diagnostics(host, port):
            lines.append(f"           {line}")
        lines.append("=> NOT READY")
        return {"connected": False, "ready": False, "config": {}, "report": "\n".join(lines)}
    lines.append(f"rosbridge: OK (ws://{host}:{port})")

    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = rb.read_config(host, port, config_topic, timeout) or {}
        except Exception as exc:
            config = {}
            lines.append(f"config:    error reading {config_topic} ({exc})")
        if config:
            allowed = bool(config.get("commands_allowed"))
            lines.append(f"config:    {config_topic} — commands_allowed: {'yes' if allowed else 'no (read-only)'}")
        else:
            lines.append(f"config:    no message on {config_topic} yet")
    lines.append("=> READY")
    return {"connected": True, "ready": True, "config": config, "report": "\n".join(lines)}


def ros2_native_status(ctx: dict) -> dict:
    """Preflight direct ROS 2 access through rclpy. Used by ROS2Status."""
    state_topic = str(ctx.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or "").strip()
    timeout = float(ctx.get("timeout") or 2.0)

    lines = [f"python:    {sys.executable} ({sys.version.split()[0]})"]
    ok, err = nr.available()
    if not ok:
        lines.append("rclpy:     MISSING")
        lines.append(f"           {err}")
        lines.append("=> NOT READY")
        return {"connected": False, "ready": False, "topics": [], "config": {}, "report": "\n".join(lines)}
    version = nr.rclpy_version()
    lines.append("rclpy:     OK" + (f" ({version})" if version else ""))

    try:
        raw_topics = nr.topic_names_and_types(timeout=min(timeout, 2.0))
    except Exception as exc:  # noqa: BLE001
        lines.append(f"native ROS 2 graph: FAILED ({type(exc).__name__}: {exc})")
        lines.append("=> NOT READY")
        return {"connected": False, "ready": False, "topics": [], "config": {}, "report": "\n".join(lines)}

    topics = [f"{name} [{', '.join(kinds)}]" for name, kinds in sorted(raw_topics)]
    topic_names = {name for name, _ in raw_topics}
    state_seen = state_topic in topic_names
    command_seen = command_topic in topic_names
    lines.append(f"native ROS 2 graph: OK ({len(topics)} topic(s))")
    lines.append(f"state topic:   {'OK' if state_seen else 'missing'} {state_topic}")
    lines.append(f"command topic: {'OK' if command_seen else 'missing'} {command_topic}")

    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = nr.read_config(config_topic, timeout=min(timeout, 2.0)) or {}
        except Exception as exc:  # noqa: BLE001
            lines.append(f"config:       {config_topic} unavailable ({type(exc).__name__}: {exc})")
        else:
            if config:
                allowed = config.get("commands_allowed")
                allowed_text = "unknown" if allowed is None else "yes" if allowed else "no"
                lines.append(f"config:       {config_topic} (commands_allowed={allowed_text})")
            else:
                lines.append(f"config:       no message on {config_topic} yet")

    ready = bool(state_seen)
    lines.append("=> READY" if ready else f"=> NOT READY: no visible {state_topic} publisher yet")
    return {"connected": True, "ready": ready, "topics": topics, "config": config, "report": "\n".join(lines)}


@node(
    name="ROS2Status", component="diagnostics",
    category=_CATEGORY,
    description="Check ROS 2 using the best available transport. Uses native rclpy when available, otherwise ensures local rosbridge.",
    inputs={
        "trigger": AnyPort,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "ensure_rosbridge": Bool(default=True),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "timeout": Float(default=10.0),
    },
    outputs={"connected": Bool, "ready": Bool, "transport": Text, "topics": List, "config": Dict, "report": Text},
)
def ros2_status(ctx: dict) -> dict:
    transport = _resolve_transport(ctx)
    prefix = _transport_report(ctx, transport)
    if transport == "native":
        result = ros2_native_status(ctx)
        return {**result, "transport": transport, "report": f"{prefix}\n{result.get('report', '')}"}

    if bool(ctx.get("ensure_rosbridge", True)):
        server = rosbridge_service.ros2_rosbridge_server({
            "action": "ensure",
            "host": ctx.get("host", "127.0.0.1"),
            "port": ctx.get("port", 9090),
            "timeout": max(30.0, float(ctx.get("timeout") or 10.0)),
        })
        if not server.get("ready"):
            return {
                "connected": False,
                "ready": False,
                "transport": transport,
                "topics": [],
                "config": {},
                "report": f"{prefix}\n{server.get('report', 'rosbridge startup failed')}",
            }
    result = ros2_rosbridge_status(ctx)
    return {
        **result,
        "transport": transport,
        "topics": [],
        "report": f"{prefix}\n{result.get('report', '')}",
    }
