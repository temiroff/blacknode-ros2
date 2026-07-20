"""Universal live robot-control nodes over ROS 2 transports.

These drive **any** robot that exposes ``sensor_msgs/msg/JointState`` over a
ROS 2 graph: native ``rclpy`` nodes for direct local/WSL control, plus
rosbridge WebSocket nodes when a WebSocket transport is preferred. Topics, joint
name, and units are all inputs, so the same nodes work for any joint-based robot
— robot specifics live in templates, not in the nodes.

Motion is gated: command nodes do nothing unless explicitly armed, sync to the
current pose before moving, clamp to limits when a config topic provides them,
and stream a heartbeat so a robot driver's own timeout still applies.
"""
from __future__ import annotations

import base64
import html
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, _NODE_REGISTRY, node

from . import ros2_native_runtime as nr
from . import rosbridge_runtime as rb
from . import rosbridge_service
from . import sample_stream

_CATEGORY = "ROS 2"
_teach_monitor_lock = threading.Lock()
_teach_monitors: dict[str, dict[str, Any]] = {}


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


def _svg_text(value: Any, limit: int = 90) -> str:
    text = " ".join(str(value or "").split())
    if len(text) > limit:
        text = text[: limit - 3] + "..."
    return html.escape(text)


def _svg_data(svg: str) -> str:
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


def _teach_dashboard(
    pose: dict[str, Any],
    units: str,
    teach_mode: bool,
    report: str = "",
    *,
    action: str = "check",
    live: bool = False,
    updated_at: str = "",
) -> str:
    accent = "#f59e0b" if teach_mode else "#22c55e"
    robot_state = "RELEASED • TORQUE OFF" if teach_mode else "HOLDING • TORQUE ON"
    control = {
        "check": "MONITOR ONLY",
        "status": "MONITOR ONLY",
        "release": "RELEASE + LIVE POSE",
        "enter": "RELEASE + LIVE POSE",
        "hold": "HOLD POSITION",
        "exit": "HOLD POSITION",
    }.get(str(action or "check").strip().lower(), str(action or "check").strip().upper())
    runtime_state = "LIVE" if live else "SNAPSHOT"
    rows = []
    for index, (name, value) in enumerate(sorted(pose.items())[:8]):
        y = 190 + index * 48
        value_text = f"{value:.2f}" if isinstance(value, (int, float)) else "-"
        rows.append(
            f'<text x="54" y="{y}" fill="#f8fafc" font-family="monospace" font-size="17">{_svg_text(name, 24)}</text>'
            f'<text x="696" y="{y}" text-anchor="end" fill="{accent}" font-family="monospace" font-size="18" font-weight="700">{value_text}</text>'
        )
    if not rows:
        rows.append('<text x="380" y="260" text-anchor="middle" fill="#93a4b8" font-family="Arial,sans-serif" font-size="18">Waiting for joint positions…</text>')
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="760" height="620" viewBox="0 0 760 620">
<rect width="760" height="620" rx="24" fill="#0b1020"/>
<rect x="24" y="24" width="712" height="112" rx="18" fill="#172033" stroke="{accent}" stroke-width="2"/>
<text x="48" y="56" fill="#f8fafc" font-family="Arial,sans-serif" font-size="23" font-weight="800">MANUAL MOVE + LIVE POSE</text>
<text x="48" y="87" fill="#cbd5e1" font-family="Arial,sans-serif" font-size="15" font-weight="700">CONTROL: {_svg_text(control, 32)} • {runtime_state}</text>
<text x="48" y="116" fill="{accent}" font-family="Arial,sans-serif" font-size="16" font-weight="800">ROBOT: {robot_state}</text>
<text x="54" y="158" fill="#93a4b8" font-family="Arial,sans-serif" font-size="12" font-weight="700">JOINT</text>
<text x="696" y="158" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="12" font-weight="700">POSITION ({_svg_text(units, 12)})</text>
{''.join(rows)}
<text x="48" y="582" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13">{_svg_text(report, 92)}</text>
<text x="712" y="582" text-anchor="end" fill="#64748b" font-family="monospace" font-size="12">{_svg_text(updated_at, 24)}</text>
</svg>"""
    return _svg_data(svg)


def _to_radians(value: float, units: str) -> float:
    return math.radians(value) if units == "degrees" else value


def _from_radians(value: float, units: str) -> float:
    return math.degrees(value) if units == "degrees" else value


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


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
    name="ROS2RosbridgeStatus",
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


@node(
    name="ROS2NativeStatus",
    category=_CATEGORY,
    hidden=True,
    description="Preflight direct ROS 2 access through rclpy: checks imports, visible topics, and optional robot config. No rosbridge required.",
    inputs={
        "trigger": AnyPort,
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "timeout": Float(default=2.0),
    },
    outputs={"connected": Bool, "ready": Bool, "topics": List, "config": Dict, "report": Text},
)
def ros2_native_status(ctx: dict) -> dict:
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
    name="ROS2Status",
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


@node(
    name="ROS2NativeRobotDiscovery",
    category=_CATEGORY,
    hidden=True,
    description="Detect a direct rclpy robot interface and return a generic robot profile. No rosbridge required.",
    inputs={
        "trigger": AnyPort,
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "timeout": Float(default=10.0),
    },
    outputs={"connected": Bool, "ready": Bool, "robot": Dict, "joints": List, "pose": Dict, "report": Text},
)
def ros2_native_robot_discovery(ctx: dict) -> dict:
    state_topic = str(ctx.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or "/joint_config").strip()
    units = str(ctx.get("units") or "degrees")
    timeout = float(ctx.get("timeout") or 10.0)
    base_robot = {
        "host": "",
        "port": 0,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "config_topic": config_topic,
        "units": units,
        "connected": False,
        "ready": False,
        "joints": [],
        "pose": {},
        "limits": {},
        "commands_allowed": None,
        "interface": {"kind": "native_ros2", "verified": False},
        "error": "",
    }

    ok, err = nr.available()
    if not ok:
        return {
            "connected": False,
            "ready": False,
            "robot": {**base_robot, "error": err},
            "joints": [],
            "pose": {},
            "report": f"native robot discovery FAILED: {err}",
        }

    config: dict[str, Any] = {}
    config_error = ""
    if config_topic:
        try:
            config = nr.read_config(config_topic, timeout) or {}
        except Exception as exc:  # keep discovery useful even if config is absent
            config_error = f"{type(exc).__name__}: {exc}"

    try:
        pose_rad = nr.read_pose(state_topic, timeout)
    except Exception as exc:
        error = f"no readable JointState on {state_topic} ({exc})"
        robot = {**base_robot, "connected": True, "config": config, "config_error": config_error, "error": error}
        return {
            "connected": True,
            "ready": False,
            "robot": robot,
            "joints": [],
            "pose": {},
            "report": f"native robot discovery FAILED: {error}",
        }

    pose = {name: _from_radians(value, units) for name, value in (pose_rad or {}).items()}
    joints = list(pose.keys())
    limits = {
        name: {"lower": _from_radians(lower, units), "upper": _from_radians(upper, units)}
        for name, (lower, upper) in nr.limits_radians(config).items()
    }
    commands_allowed = config.get("commands_allowed") if "commands_allowed" in config else None
    ready = bool(joints)
    robot = {
        **base_robot,
        "connected": True,
        "ready": ready,
        "joints": joints,
        "pose": pose,
        "limits": limits,
        "commands_allowed": commands_allowed,
        "config": config,
        "config_error": config_error,
        "interface": {"kind": "native_ros2", "verified": ready},
    }

    lines = ["native ROS 2: rclpy direct"]
    lines.append(f"joint state: {state_topic} ({len(joints)} joint(s))")
    lines.append(f"command topic: {command_topic}")
    if config_topic:
        if config:
            allowed_text = "unknown" if commands_allowed is None else "yes" if commands_allowed else "no"
            lines.append(f"config: {config_topic} (commands_allowed={allowed_text}, limits={len(limits)})")
        elif config_error:
            lines.append(f"config: {config_topic} unavailable ({config_error})")
        else:
            lines.append(f"config: no message on {config_topic} yet")
    if joints:
        lines.append("joints: " + ", ".join(joints[:10]) + (" ..." if len(joints) > 10 else ""))
        lines.append("=> READY")
    else:
        robot["error"] = f"no JointState on {state_topic} within {timeout:g}s"
        lines.append(f"=> NOT READY: {robot['error']}")
    return {"connected": True, "ready": ready, "robot": robot, "joints": joints, "pose": pose, "report": "\n".join(lines)}


@node(
    name="ROS2NativeJointState",
    category=_CATEGORY,
    hidden=True,
    description="Read the current pose from any JointState topic through native rclpy (radians, or degrees). No rosbridge required.",
    inputs={
        "trigger": AnyPort,
        "topic": Text(default="/joint_states"),
        "units": Enum(["radians", "degrees"], default="radians"),
        "timeout": Float(default=10.0),
    },
    outputs={"pose": Dict, "names": List, "report": Text},
)
def ros2_native_joint_state(ctx: dict) -> dict:
    ok, err = nr.available()
    if not ok:
        return {"pose": {}, "names": [], "report": f"native joint state FAILED: {err}"}
    topic = str(ctx.get("topic") or "/joint_states")
    units = str(ctx.get("units") or "radians")
    timeout = float(ctx.get("timeout") or 10.0)
    try:
        pose_rad = nr.read_pose(topic, timeout)
    except Exception as exc:
        return {"pose": {}, "names": [], "report": f"native joint state FAILED: {exc}"}
    if not pose_rad:
        return {"pose": {}, "names": [], "report": f"no JointState on {topic} within {timeout:g}s - is the ROS 2 robot driver running?"}
    pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
    summary = ", ".join(f"{name} {pose[name]:.2f}" for name in pose)
    return {"pose": pose, "names": list(pose.keys()), "report": f"{len(pose)} joints ({units}) via native rclpy: {summary}"}


@node(
    name="ROS2NativeSetJoint",
    category=_CATEGORY,
    hidden=True,
    description="Set one joint to an absolute position through native rclpy. Safe by default: only streams commands when armed.",
    inputs={
        "trigger": AnyPort,
        "robot": Dict,
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "joint": Text(default=""),
        "position": Float(default=0.0),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "ramp_seconds": Float(default=0.8),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={"moved": Bool, "joint": Text, "before": Dict, "after": Dict, "target": Dict, "report": Text},
)
def ros2_native_set_joint(ctx: dict) -> dict:
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    armed = bool(ctx.get("armed", False))
    blocked = {"moved": False, "joint": joint, "before": {}, "after": {}, "target": {}}
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to a joint name (discover them with ROS2NativeJointState)."}
    ok, err = nr.available()
    if not ok:
        return {**blocked, "report": f"native set {joint} FAILED: {err}"}

    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    position = float(ctx.get("position") or 0.0)
    ramp_seconds = float(ctx.get("ramp_seconds") or 0.8)
    hold_seconds = float(ctx.get("hold_seconds") or 0.2)
    rate_hz = float(ctx.get("rate_hz") or 30.0)
    timeout = float(ctx.get("timeout") or 10.0)

    # Reading pose/config is a passive subscribe -- no motor command is ever
    # sent by it -- so it happens regardless of `armed`. This is what lets a
    # disarmed preview show real numbers instead of empty dicts. The only
    # operation actually gated behind `armed` below is nr.stream_motion(),
    # the one call that writes to the command topic.
    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = nr.read_config(config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "report": f"native set {joint} FAILED: {exc}"}

    try:
        start_rad = nr.read_pose(state_topic, timeout)
    except Exception as exc:
        return {**blocked, "report": f"native set {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "report": f"native set {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {**blocked, "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}"}

    names = list(start_rad.keys())
    raw_target_rad = _to_radians(position, units)
    limits = nr.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_rad_value = min(upper, max(lower, raw_target_rad))
    else:
        target_rad_value = raw_target_rad
    target_rad = dict(start_rad)
    target_rad[joint] = target_rad_value

    before = {n: _from_radians(v, units) for n, v in start_rad.items()}
    target = {n: _from_radians(v, units) for n, v in target_rad.items()}
    clamp_note = "" if abs(raw_target_rad - target_rad_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"

    if not armed:
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": (
                f"PREVIEW (not armed): {joint} currently {before[joint]:.2f} {units}, "
                f"would move to {target[joint]:.2f}{clamp_note}. Set armed=true to actually move it."
            ),
        }

    if config and "commands_allowed" in config and not bool(config.get("commands_allowed")):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": "BLOCKED: the robot driver reports it is read-only (commands_allowed=false).",
        }

    result = nr.stream_motion(
        command_topic, names, start_rad, target_rad,
        ramp_seconds=ramp_seconds, hold_seconds=hold_seconds, rate_hz=rate_hz, timeout=timeout,
    )
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": f"native set {joint} FAILED: {result.get('error', 'unknown error')}",
        }

    try:
        after_rad = nr.read_pose(state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {n: _from_radians(v, units) for n, v in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    report = (
        f"native set {joint}: {before[joint]:.2f} -> {after.get(joint, before[joint]):.2f} {units} "
        f"(target {target[joint]:.2f}{clamp_note}); streamed {result.get('sent', 0)} commands at {rate_hz:g} Hz"
    )
    return {"moved": moved, "joint": joint, "before": before, "after": after, "target": target, "report": report}


@node(
    name="ROS2SetJoint",
    category=_CATEGORY,
    description="Set one joint to an absolute position using native ROS 2 or rosbridge automatically. Safe by default: disarmed.",
    inputs={
        "trigger": AnyPort,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "manual_action": Text(default="check"),
        "joint": Text(default=""),
        "position": Float(default=0.0),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "ramp_seconds": Float(default=0.8),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={"moved": Bool, "joint": Text, "before": Dict, "after": Dict, "target": Dict, "report": Text},
)
def ros2_set_joint(ctx: dict) -> dict:
    manual_action = str(ctx.get("manual_action") or ctx.get("teach_action") or "check").strip().lower()
    if manual_action not in {"status", "check"}:
        return {
            "moved": False,
            "joint": str(ctx.get("joint") or "").strip(),
            "before": {},
            "after": {},
            "target": {},
            "report": f"BLOCKED: manual-move action '{manual_action}' was requested in this run; recook with action=check before commanding motion.",
        }
    transport = _resolve_transport(ctx)
    if transport == "native":
        result = ros2_native_set_joint(ctx)
        result["report"] = f"{_transport_report(ctx, transport)}\n{result.get('report', '')}"
        return result

    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    blocked = {"moved": False, "joint": joint, "before": {}, "after": {}, "target": {}}
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to a joint name (discover them with ROS2JointState)."}

    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    position = float(ctx.get("position") or 0.0)
    ramp_seconds = float(ctx.get("ramp_seconds") or 0.8)
    hold_seconds = float(ctx.get("hold_seconds") or 0.2)
    rate_hz = float(ctx.get("rate_hz") or 30.0)
    timeout = float(ctx.get("timeout") or 10.0)

    ok, err = rb.available()
    if not ok:
        return {**blocked, "report": f"rosbridge set {joint} FAILED: {err}"}
    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = rb.read_config(host, port, config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "report": f"rosbridge set {joint} FAILED: {exc}"}
    try:
        start_rad = rb.read_pose(host, port, state_topic, timeout)
    except Exception as exc:
        return {**blocked, "report": f"rosbridge set {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "report": f"rosbridge set {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {**blocked, "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}"}

    raw_target_rad = _to_radians(position, units)
    target_value = raw_target_rad
    limits = rb.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_value = min(upper, max(lower, raw_target_rad))
    target_rad = dict(start_rad)
    target_rad[joint] = target_value
    before = {name: _from_radians(value, units) for name, value in start_rad.items()}
    target = {name: _from_radians(value, units) for name, value in target_rad.items()}
    clamp_note = "" if abs(raw_target_rad - target_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"

    if not bool(ctx.get("armed", False)):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": (
                f"{_transport_report(ctx, transport)}\nPREVIEW (not armed): {joint} currently {before[joint]:.2f} {units}, "
                f"would move to {target[joint]:.2f}{clamp_note}. Set armed=true to move."
            ),
        }
    if config and config.get("commands_allowed") is False:
        return {**blocked, "before": before, "after": before, "target": target, "report": "BLOCKED: robot reports commands_allowed=false."}

    result = rb.stream_motion(
        host,
        port,
        command_topic,
        list(start_rad),
        start_rad,
        target_rad,
        ramp_seconds=ramp_seconds,
        hold_seconds=hold_seconds,
        rate_hz=rate_hz,
        timeout=timeout,
    )
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": f"rosbridge set {joint} FAILED: {result.get('error', 'unknown error')}",
        }
    try:
        after_rad = rb.read_pose(host, port, state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {name: _from_radians(value, units) for name, value in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    return {
        "moved": moved,
        "joint": joint,
        "before": before,
        "after": after,
        "target": target,
        "report": (
            f"{_transport_report(ctx, transport)}\nrosbridge set {joint}: {before[joint]:.2f} -> "
            f"{after.get(joint, before[joint]):.2f} {units} (target {target[joint]:.2f}{clamp_note})"
        ),
    }


@node(
    name="ROS2RobotDiscovery",
    category=_CATEGORY,
    description="Detect a robot over native ROS 2 or rosbridge automatically and return one generic robot profile.",
    inputs={
        "trigger": AnyPort,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "timeout": Float(default=10.0),
    },
    outputs={"connected": Bool, "ready": Bool, "robot": Dict, "joints": List, "pose": Dict, "report": Text},
)
def ros2_robot_discovery(ctx: dict) -> dict:
    transport = _resolve_transport(ctx)
    if transport == "native":
        result = ros2_native_robot_discovery(ctx)
        result["report"] = f"{_transport_report(ctx, transport)}\n{result.get('report', '')}"
        return result
    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or "/joint_config").strip()
    units = str(ctx.get("units") or "degrees")
    timeout = float(ctx.get("timeout") or 10.0)
    base_robot = {
        "host": host,
        "port": port,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "config_topic": config_topic,
        "units": units,
        "connected": False,
        "ready": False,
        "joints": [],
        "pose": {},
        "limits": {},
        "commands_allowed": None,
        "error": "",
        "diagnostics": [],
        "interface": {"kind": "rosbridge", "verified": False},
    }

    ok, err = rb.available()
    if not ok:
        return {
            "connected": False,
            "ready": False,
            "robot": {**base_robot, "error": err},
            "joints": [],
            "pose": {},
            "report": f"robot discovery FAILED: {err}",
        }

    lines = [f"rosbridge: ws://{host}:{port}"]
    try:
        rb.get_connection(host, port, timeout)
    except Exception as exc:
        error = f"could not connect to ws://{host}:{port} ({exc})"
        diagnostics = _rosbridge_connection_diagnostics(host, port)
        return {
            "connected": False,
            "ready": False,
            "robot": {**base_robot, "error": error, "diagnostics": diagnostics},
            "joints": [],
            "pose": {},
            "report": "\n".join([f"robot discovery FAILED: {error}", *diagnostics]),
        }

    config: dict[str, Any] = {}
    config_error = ""
    if config_topic:
        try:
            config = rb.read_config(host, port, config_topic, timeout) or {}
        except Exception as exc:  # keep discovery useful even if config is absent
            config_error = f"{type(exc).__name__}: {exc}"

    try:
        pose_rad = rb.read_pose(host, port, state_topic, timeout)
    except Exception as exc:
        error = f"no readable JointState on {state_topic} ({exc})"
        robot = {**base_robot, "connected": True, "config": config, "config_error": config_error, "error": error}
        return {
            "connected": True,
            "ready": False,
            "robot": robot,
            "joints": [],
            "pose": {},
            "report": f"robot discovery FAILED: {error}",
        }

    pose = {name: _from_radians(value, units) for name, value in (pose_rad or {}).items()}
    joints = list(pose.keys())
    limits = {
        name: {"lower": _from_radians(lower, units), "upper": _from_radians(upper, units)}
        for name, (lower, upper) in rb.limits_radians(config).items()
    }
    commands_allowed = config.get("commands_allowed") if "commands_allowed" in config else None
    ready = bool(joints)
    robot = {
        **base_robot,
        "connected": True,
        "ready": ready,
        "joints": joints,
        "pose": pose,
        "limits": limits,
        "commands_allowed": commands_allowed,
        "config": config,
        "config_error": config_error,
        "interface": {"kind": "rosbridge", "verified": ready},
    }
    lines.append(f"joint state: {state_topic} ({len(joints)} joint(s))")
    lines.append(f"command topic: {command_topic}")
    if config_topic:
        if config:
            allowed_text = "unknown" if commands_allowed is None else "yes" if commands_allowed else "no"
            lines.append(f"config: {config_topic} (commands_allowed={allowed_text}, limits={len(limits)})")
        elif config_error:
            lines.append(f"config: {config_topic} unavailable ({config_error})")
        else:
            lines.append(f"config: no message on {config_topic} yet")
    if joints:
        lines.append("joints: " + ", ".join(joints[:10]) + (" ..." if len(joints) > 10 else ""))
        lines.append("=> READY")
    else:
        robot["error"] = f"no JointState on {state_topic} within {timeout:g}s"
        lines.append(f"=> NOT READY: {robot['error']}")
    return {"connected": True, "ready": ready, "robot": robot, "joints": joints, "pose": pose, "report": "\n".join(lines)}


@node(
    name="ROS2JointState",
    category=_CATEGORY,
    description="Read joint state using native rclpy when available, otherwise rosbridge.",
    inputs={
        "trigger": AnyPort,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "topic": Text(default="/joint_states"),
        "units": Enum(["radians", "degrees"], default="radians"),
        "timeout": Float(default=10.0),
    },
    outputs={"pose": Dict, "names": List, "report": Text},
)
def ros2_joint_state(ctx: dict) -> dict:
    transport = _resolve_transport(ctx)
    if transport == "native":
        result = ros2_native_joint_state(ctx)
        result["report"] = f"{_transport_report(ctx, transport)}\n{result.get('report', '')}"
        return result
    ok, err = rb.available()
    if not ok:
        return {"pose": {}, "names": [], "report": f"joint state FAILED: {err}"}
    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)
    topic = str(ctx.get("topic") or "/joint_states")
    units = str(ctx.get("units") or "radians")
    timeout = float(ctx.get("timeout") or 10.0)
    try:
        pose_rad = rb.read_pose(host, port, topic, timeout)
    except Exception as exc:
        return {"pose": {}, "names": [], "report": f"joint state FAILED: {exc}"}
    if not pose_rad:
        return {"pose": {}, "names": [], "report": f"no JointState on {topic} within {timeout:g}s — is the robot bridge running?"}
    pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
    summary = ", ".join(f"{name} {pose[name]:.2f}" for name in pose)
    return {
        "pose": pose,
        "names": list(pose.keys()),
        "report": f"{_transport_report(ctx, transport)}\n{len(pose)} joints ({units}): {summary}",
    }


def _live_motion_dashboard_outputs(
    ctx: dict[str, Any], pose: dict[str, float], units: str, torque_enabled: bool, item: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    """Push a live pose into supported directly connected pass-through nodes."""
    graph = ctx.get("__graph__")
    source_id = str(ctx.get("__node_id__") or "")
    if graph is None or not source_id or not pose:
        return {}
    outputs: dict[str, dict[str, Any]] = {}
    for edge in list(getattr(graph, "_edges", []) or []):
        if edge.get("from") != source_id or edge.get("from_port") != "pose":
            continue
        target_id = str(edge.get("to") or "")
        target = getattr(graph, "_nodes", {}).get(target_id) or {}
        target_type = str(target.get("type") or "")
        if target_type == "ROS2MotionDashboard":
            baselines = item.setdefault("dashboard_baselines", {})
            baseline = baselines.setdefault(target_id, dict(pose))
            dashboard_ctx = dict(target.get("params") or {})
            dashboard_ctx.update({
                "pose": dict(pose),
                "before": dict(baseline),
                "units": units,
                "__live_pose__": True,
                "__torque_enabled__": torque_enabled,
                "__manual_mode__": "holding" if torque_enabled else "released",
            })
            try:
                outputs[target_id] = dict(ros2_motion_dashboard(dashboard_ctx))
            except Exception:
                continue
        elif target_type == "RobotConnectionDashboard":
            dashboard_fn = _NODE_REGISTRY.get(target_type)
            if dashboard_fn is None:
                continue
            dashboard_ctx = dict(target.get("params") or {})
            cache = getattr(graph, "_cache", {}) or {}
            for incoming in list(getattr(graph, "_edges", []) or []):
                if incoming.get("to") != target_id or incoming.get("to_port") == "pose":
                    continue
                cache_key = (incoming.get("from"), incoming.get("from_port"))
                if cache_key in cache:
                    dashboard_ctx[str(incoming.get("to_port") or "")] = cache[cache_key]
            dashboard_ctx["pose"] = dict(pose)
            try:
                outputs[target_id] = dict(dashboard_fn(dashboard_ctx))
            except Exception:
                continue
        elif target_type == "RobotCalibrationRecorder":
            calibration_fn = _NODE_REGISTRY.get(target_type)
            if calibration_fn is None:
                continue
            calibration_ctx = dict(target.get("params") or {})
            # Reuse values resolved during the original graph cook (profile,
            # hardware serial, and safety options) while replacing only the
            # live pose and torque state.
            cache = getattr(graph, "_cache", {}) or {}
            for incoming in list(getattr(graph, "_edges", []) or []):
                if incoming.get("to") != target_id or incoming.get("to_port") == "pose":
                    continue
                cache_key = (incoming.get("from"), incoming.get("from_port"))
                if cache_key in cache:
                    calibration_ctx[str(incoming.get("to_port") or "")] = cache[cache_key]
            calibration_ctx.update({
                "action": "_sample",
                "pose": dict(pose),
                "torque_enabled": torque_enabled,
                "__live_pose__": True,
            })
            try:
                outputs[target_id] = dict(calibration_fn(calibration_ctx))
            except Exception:
                continue
    return outputs


def _teach_monitor_worker(run_id: str, item: dict[str, Any]) -> None:
    while not item["stop"].wait(0.1):
        ctx = dict(item["ctx"])
        robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
        transport = _resolve_transport(ctx)
        host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
        port = int(ctx.get("port") or robot.get("port") or 9090)
        state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
        config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "/joint_config")
        command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
        units = str(ctx.get("units") or robot.get("units") or "degrees")
        try:
            if transport == "native":
                config = nr.read_config(config_topic, 0.15) or {}
                pose_rad = nr.read_pose(state_topic, 0.15) or {}
            else:
                session = item.get("session")
                if session is None:
                    session = rb.acquire_joint_stream(host, port, state_topic, command_topic, config_topic, timeout=2.0)
                    item["session"] = session
                    session.wait_for_pose(1.0)
                    session.wait_for_config(0.5)
                confirmed_config = item.pop("confirmed_config", None)
                if isinstance(confirmed_config, dict):
                    session.seed_config(confirmed_config)
                pose_rad, config, state_age = session.snapshot()
                if state_age > 2.0:
                    # The worker heartbeat is not proof that rosbridge is
                    # still delivering JointState callbacks. Replace a stale
                    # Topic immediately so mode changes recover without a
                    # graph or Blacknode restart.
                    rb.release_joint_stream(session, discard=True)
                    item["session"] = None
                    item["error"] = f"joint-state subscription stale ({state_age:.1f}s); reconnecting"
                    continue
                if not config:
                    previous = item.get("outputs") or {}
                    config = {"torque_enabled": previous.get("torque_enabled", False)}
            torque_enabled = bool(config.get("torque_enabled", False))
            pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
            command_error = str(ctx.get("__manual_command_error__") or "")
            report = command_error or ("Move the supported arm by hand; live joint positions update here." if not torque_enabled else "Holding current pose.")
            updated_at = time.strftime("updated %H:%M:%S")
            outputs = {
                "action": str(ctx.get("action") or "check"),
                "live": True,
                "data_ready": bool(pose),
                "mode": "released" if not torque_enabled else "hold",
                "torque_enabled": torque_enabled,
                "command_ok": not bool(command_error),
                "pose": pose,
                "joints": list(pose),
                "updated_at": updated_at,
                "dashboard": _teach_dashboard(pose, units, not torque_enabled, report, action=str(ctx.get("action") or "check"), live=True, updated_at=updated_at),
                "report": report,
            }
            downstream_outputs = _live_motion_dashboard_outputs(ctx, pose, units, torque_enabled, item)
            with _teach_monitor_lock:
                current = _teach_monitors.get(run_id)
                if current is not item:
                    return
                item["outputs"] = outputs
                item["downstream_outputs"] = downstream_outputs
                item["downstream_types"] = {
                    str(edge.get("to") or ""): str((getattr(ctx.get("__graph__"), "_nodes", {}).get(str(edge.get("to") or "")) or {}).get("type") or "")
                    for edge in list(getattr(ctx.get("__graph__"), "_edges", []) or [])
                    if edge.get("from") == str(ctx.get("__node_id__") or "") and edge.get("from_port") == "pose"
                }
                item["updated_at"] = time.time()
                item["error"] = ""
        except Exception as exc:
            with _teach_monitor_lock:
                if _teach_monitors.get(run_id) is not item:
                    return
                item["error"] = f"{type(exc).__name__}: {exc}"


def _start_teach_monitor(
    run_id: str,
    ctx: dict,
    outputs: dict[str, Any],
    confirmed_config: dict[str, Any] | None = None,
) -> None:
    with _teach_monitor_lock:
        existing = _teach_monitors.get(run_id)
        if existing is not None:
            existing["ctx"] = dict(ctx)
            existing["outputs"] = dict(outputs)
            existing["dashboard_baselines"] = {}
            existing["confirmed_config"] = dict(confirmed_config or {})
            session = existing.get("session")
            if session is not None and confirmed_config:
                session.seed_config(confirmed_config)
            return
        item: dict[str, Any] = {
            "ctx": dict(ctx),
            "outputs": dict(outputs),
            "updated_at": time.time(),
            "error": "",
            "downstream_outputs": {},
            "dashboard_baselines": {},
            "confirmed_config": dict(confirmed_config or {}),
            "stop": threading.Event(),
        }
        _teach_monitors[run_id] = item
    thread = threading.Thread(target=_teach_monitor_worker, args=(run_id, item), name=f"blacknode-teach-{run_id}", daemon=True)
    item["thread"] = thread
    thread.start()


def _stop_teach_monitor(run_id: str) -> None:
    with _teach_monitor_lock:
        item = _teach_monitors.pop(run_id, None)
    if item is not None:
        item["stop"].set()
        rb.release_joint_stream(item.get("session"))


def runtime_status() -> dict[str, Any]:
    with _teach_monitor_lock:
        monitors = []
        for run_id, item in _teach_monitors.items():
            monitors.append({
                "run_id": run_id,
                "node_id": str(item.get("ctx", {}).get("__node_id__") or ""),
                "node_type": "ROS2ManualMove",
                "outputs": dict(item.get("outputs") or {}),
                "updated_at": item.get("updated_at"),
                "error": item.get("error") or "",
            })
            for node_id, outputs in dict(item.get("downstream_outputs") or {}).items():
                monitors.append({
                    "run_id": run_id,
                    "node_id": node_id,
                    "node_type": str(item.get("downstream_types", {}).get(node_id) or "live downstream"),
                    "outputs": dict(outputs or {}),
                    "updated_at": item.get("updated_at"),
                    "error": "",
                })
    try:
        from blacknode.pkg.blacknode_skills.follow_person.leader_follower_runtime import monitor_entries
        leader_follower_entries = monitor_entries()
    except Exception:
        leader_follower_entries = []
    monitors.extend(leader_follower_entries)
    return {
        "ok": True,
        "active": bool(monitors),
        "node_outputs": monitors,
        "managed_runs": (
            [{"run_id": run_id, "kind": "manual_move"} for run_id in _teach_monitors]
            + [{"run_id": entry["run_id"], "kind": "leader_follower"} for entry in leader_follower_entries]
        ),
        "report": f"{len(_teach_monitors)} manual-move monitor(s), {len(leader_follower_entries)} leader-follower controller(s) active",
    }


def stop_runtime_services() -> dict[str, Any]:
    with _teach_monitor_lock:
        items = list(_teach_monitors.values())
        _teach_monitors.clear()
    for item in items:
        item["stop"].set()
        rb.release_joint_stream(item.get("session"))
    try:
        from blacknode.pkg.blacknode_skills.follow_person.follow_runtime import stop_continuous_follow_services
        follow_result = stop_continuous_follow_services()
    except ModuleNotFoundError:
        follow_result = {"ok": True, "stopped": 0, "error": ""}
    except Exception as exc:
        follow_result = {"ok": False, "stopped": 0, "error": str(exc)}
    stopped_follow = int(follow_result.get("stopped") or 0)
    try:
        from blacknode.pkg.blacknode_skills.follow_person.leader_follower_runtime import stop_leader_follower_services
        leader_follower_result = stop_leader_follower_services()
    except ModuleNotFoundError:
        leader_follower_result = {"ok": True, "stopped": 0, "error": ""}
    except Exception as exc:
        leader_follower_result = {"ok": False, "stopped": 0, "error": str(exc)}
    stopped_leader_follower = int(leader_follower_result.get("stopped") or 0)
    closed_sessions = rb.close_joint_streams()
    return {
        "ok": bool(follow_result.get("ok", True) and leader_follower_result.get("ok", True)),
        "stopped": {
            "streams": len(items),
            "managed_runs": stopped_follow + stopped_leader_follower,
            "detached": 0,
            "joint_streams": closed_sessions,
        },
        "report": (
            f"stopped {len(items)} manual-move monitor(s), {stopped_follow} follow controller(s), "
            f"and {stopped_leader_follower} leader-follower controller(s); "
            f"closed {closed_sessions} joint stream(s)"
        ),
    }


@node(
    name="ROS2ManualMove",
    category=_CATEGORY,
    live=True,
    description="Release torque for safe hand positioning or hold the current pose, with an explicit live joint monitor.",
    inputs={
        "trigger": AnyPort,
        "run_id": Text(default="robot_teach"),
        "action": Enum(["check", "release", "hold"], default="check"),
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "config_topic": Text(default="/joint_config"),
        "control_topic": Text(default="/robot_control"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "timeout": Float(default=5.0),
    },
    outputs={
        "action": Text,
        "live": Bool,
        "data_ready": Bool,
        "mode": Text,
        "torque_enabled": Bool,
        "command_ok": Bool,
        "pose": Dict,
        "joints": List,
        "updated_at": Text,
        "dashboard": Image,
        "report": Text,
    },
)
def ros2_manual_move(ctx: dict) -> dict:
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    raw_action = str(ctx.get("action") or "check").strip().lower()
    action = {"status": "check", "enter": "release", "exit": "hold"}.get(raw_action, raw_action)
    run_id = str(ctx.get("run_id") or "robot_teach").strip() or "robot_teach"
    transport = _resolve_transport(ctx)
    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "/joint_config")
    control_topic = str(ctx.get("control_topic") or robot.get("control_topic") or "/robot_control")
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    timeout = max(0.5, float(ctx.get("timeout") or 5.0))
    # Direct/MCP calls predate graph execution modes and retain their live
    # behavior. The editor/server always supplies an explicit mode.
    run_mode = "once" if ctx.get("__run_mode__") == "once" else "live"
    base = {
        "action": action,
        "live": False,
        "data_ready": False,
        "mode": "unknown",
        "torque_enabled": False,
        "command_ok": False,
        "pose": {},
        "joints": [],
        "updated_at": "not run yet",
        "dashboard": _teach_dashboard({}, units, False, "Waiting for robot state", action=action),
    }

    if transport == "native":
        ok, error = nr.available()
        read_config = lambda wait: nr.read_config(config_topic, wait)
        read_pose = lambda wait: nr.read_pose(state_topic, wait)
        publish = lambda payload: nr.publish_string(control_topic, payload, min(timeout, 2.0))
    else:
        ok, error = rb.available()
        read_config = lambda wait: rb.read_config(host, port, config_topic, wait)
        read_pose = lambda wait: rb.read_pose(host, port, state_topic, wait)
        publish = lambda payload: rb.publish_string(host, port, control_topic, payload, min(timeout, 2.0))
    if not ok:
        return {**base, "report": f"manual move FAILED: {error}"}

    if action in {"release", "hold"}:
        requested = "enter_teach" if action == "release" else "exit_teach"
        published = publish(json.dumps({"action": requested}))
        if not published.get("ok"):
            return {**base, "report": f"manual move FAILED: {published.get('error', 'control command was not accepted')}"}

    desired_torque = action == "hold" if action != "check" else None
    deadline = time.monotonic() + timeout
    config: dict[str, Any] = {}
    while time.monotonic() < deadline:
        try:
            config = read_config(min(0.5, max(0.05, deadline - time.monotonic()))) or {}
        except Exception as exc:
            if action == "check":
                return {**base, "report": f"live pose check FAILED: {type(exc).__name__}: {exc}"}
        if "torque_enabled" in config and (
            desired_torque is None or bool(config.get("torque_enabled")) == desired_torque
        ):
            break
        time.sleep(0.05)

    if "torque_enabled" not in config:
        return {
            **base,
            "report": f"manual move unsupported: no torque state on {config_topic}; restart with the updated robot driver.",
        }
    torque_enabled = bool(config.get("torque_enabled"))
    teach_mode = not torque_enabled
    try:
        pose_rad = read_pose(min(timeout, 2.0)) or {}
    except Exception:
        pose_rad = {}
    pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
    last_error = str(config.get("last_error") or "")
    acknowledged = desired_torque is None or torque_enabled == desired_torque

    if not acknowledged:
        report = f"manual move FAILED: driver did not acknowledge '{action}' within {timeout:g}s"
    elif last_error:
        report = f"manual move WARNING: {last_error}; keep the arm supported and use Stop all if any joint still resists."
    elif teach_mode and run_mode == "live":
        report = (
            f"RELEASED: torque is off; support the arm and move it by hand. "
            f"LIVE monitor is reading {len(pose)} joint position(s) from {state_topic}."
        )
    elif teach_mode:
        report = "RELEASED: torque is off. This is a one-time pose snapshot; use Go live to watch hand movement."
    elif run_mode == "live":
        report = "HOLDING: live pose monitoring is active; torque is on. Release only when the arm is supported."
    else:
        report = "HOLDING: torque is on. This is a one-time pose snapshot; use Go live for continuous updates."
    updated_at = time.strftime("updated %H:%M:%S")
    is_live = run_mode == "live"
    result = {
        "action": action,
        "live": is_live,
        "data_ready": bool(pose),
        "mode": "released" if teach_mode else "hold",
        "torque_enabled": torque_enabled,
        "command_ok": acknowledged,
        "pose": pose,
        "joints": list(pose),
        "updated_at": updated_at if is_live else f"snapshot {time.strftime('%H:%M:%S')}",
        "dashboard": _teach_dashboard(pose, units, teach_mode, report, action=action, live=is_live, updated_at=updated_at if is_live else "ONE-TIME SNAPSHOT"),
        "report": f"{_transport_report(ctx, transport)}\n{report}",
    }
    if is_live:
        monitor_ctx = {
            **ctx,
            "action": action,
            "__manual_command_error__": "" if acknowledged else report,
        }
        _start_teach_monitor(run_id, monitor_ctx, result, config)
    else:
        _stop_teach_monitor(run_id)
    return result


@node(
    name="ROS2TeachMode",
    category=_CATEGORY,
    hidden=True,
    live=True,
    description="Compatibility alias for ROS2ManualMove.",
    inputs={
        "trigger": AnyPort,
        "run_id": Text(default="robot_teach"),
        "action": Text(default="status"),
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "config_topic": Text(default="/joint_config"),
        "control_topic": Text(default="/robot_control"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "timeout": Float(default=5.0),
    },
    outputs={"action": Text, "live": Bool, "data_ready": Bool, "mode": Text, "torque_enabled": Bool, "command_ok": Bool, "pose": Dict, "joints": List, "updated_at": Text, "dashboard": Image, "report": Text},
)
def ros2_teach_mode_alias(ctx: dict) -> dict:
    return ros2_manual_move(ctx)


@node(
    name="ROS2RotateJoint",
    category=_CATEGORY,
    description="Move one joint by a relative delta using native ROS 2 or rosbridge automatically. Safe by default: disarmed.",
    inputs={
        "trigger": AnyPort,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "joint": Text(default=""),
        "delta": Float(default=10.0),
        "units": Enum(["radians", "degrees"], default="radians"),
        "ramp_seconds": Float(default=1.5),
        "hold_seconds": Float(default=2.0),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={"moved": Bool, "joint": Text, "before": Dict, "after": Dict, "target": Dict, "report": Text},
)
def ros2_rotate_joint(ctx: dict) -> dict:
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or "radians")
    blocked = {"moved": False, "joint": joint, "before": {}, "after": {}, "target": {}}
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to a joint name (discover them with ROS2JointState)."}
    transport = _resolve_transport(ctx)
    if transport == "native":
        state_topic = str(ctx.get("state_topic") or "/joint_states")
        timeout = float(ctx.get("timeout") or 10.0)
        try:
            pose_rad = nr.read_pose(state_topic, timeout)
        except Exception as exc:
            return {**blocked, "report": f"native rotate {joint} FAILED: {exc}"}
        if not pose_rad or joint not in pose_rad:
            return {**blocked, "report": f"BLOCKED: joint '{joint}' is not available on {state_topic}."}
        position = _from_radians(pose_rad[joint], units) + float(ctx.get("delta") or 0.0)
        result = ros2_native_set_joint({**ctx, "position": position})
        result["report"] = f"{_transport_report(ctx, transport)}\n{result.get('report', '')}"
        return result
    if not bool(ctx.get("armed", False)):
        return {
            **blocked,
            "report": (
                "BLOCKED: motion preview only. To move the real robot, set armed=true "
                "(and make sure the robot bridge is running and accepts commands)."
            ),
        }
    ok, err = rb.available()
    if not ok:
        return {**blocked, "report": f"rotate {joint} FAILED: {err}"}

    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or "").strip()
    delta = float(ctx.get("delta") or 0.0)
    ramp_seconds = float(ctx.get("ramp_seconds") or 1.5)
    hold_seconds = float(ctx.get("hold_seconds") or 2.0)
    rate_hz = float(ctx.get("rate_hz") or 30.0)
    timeout = float(ctx.get("timeout") or 10.0)

    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = rb.read_config(host, port, config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "report": f"rotate {joint} FAILED: {exc}"}
        if config and "commands_allowed" in config and not bool(config.get("commands_allowed")):
            return {
                **blocked,
                "report": "BLOCKED: the robot bridge reports it is read-only (commands_allowed=false). Relaunch it to accept commands.",
            }

    try:
        start_rad = rb.read_pose(host, port, state_topic, timeout)
    except Exception as exc:
        return {**blocked, "report": f"rotate {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "report": f"rotate {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {**blocked, "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}"}

    names = list(start_rad.keys())
    delta_rad = _to_radians(delta, units)
    limits = rb.limits_radians(config)
    raw_target_rad = start_rad[joint] + delta_rad
    if joint in limits:
        lower, upper = limits[joint]
        target_rad_value = min(upper, max(lower, raw_target_rad))
    else:
        target_rad_value = raw_target_rad
    target_rad = dict(start_rad)
    target_rad[joint] = target_rad_value

    result = rb.stream_motion(
        host, port, command_topic, names, start_rad, target_rad,
        ramp_seconds=ramp_seconds, hold_seconds=hold_seconds, rate_hz=rate_hz, timeout=timeout,
    )
    if not result.get("ok"):
        before = {n: _from_radians(v, units) for n, v in start_rad.items()}
        return {
            "moved": False, "joint": joint, "before": before, "after": before,
            "target": {n: _from_radians(v, units) for n, v in target_rad.items()},
            "report": f"rotate {joint} FAILED: {result.get('error', 'unknown error')}",
        }

    try:
        after_rad = rb.read_pose(host, port, state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)

    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    before = {n: _from_radians(v, units) for n, v in start_rad.items()}
    after = {n: _from_radians(v, units) for n, v in after_rad.items()}
    target = {n: _from_radians(v, units) for n, v in target_rad.items()}
    clamp_note = "" if abs(raw_target_rad - target_rad_value) < 1e-9 else f" (clamped from {_from_radians(raw_target_rad, units):.2f})"
    report = (
        f"rotate {joint}: {before[joint]:.2f} -> {after.get(joint, before[joint]):.2f} {units} "
        f"(target {target[joint]:.2f}{clamp_note}); streamed {result.get('sent', 0)} commands at {rate_hz:g} Hz"
    )
    return {"moved": moved, "joint": joint, "before": before, "after": after, "target": target, "report": report}


@node(
    name="ROS2MotionDashboard",
    category=_CATEGORY,
    live=True,
    description="Render live pose updates when connected to Manual Move, or a one-time before/after motion result.",
    inputs={
        "joint": Text(default=""),
        "pose": Dict,
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "moved": Bool(default=False),
        "units": Text(default="radians"),
    },
    outputs={"dashboard": Image, "live": Bool, "summary": Dict},
)
def ros2_motion_dashboard(ctx: dict) -> dict:
    live_pose = bool(ctx.get("__live_pose__"))
    torque_enabled = bool(ctx.get("__torque_enabled__")) if live_pose else None
    manual_mode = str(ctx.get("__manual_mode__") or ("holding" if torque_enabled else "released")) if live_pose else "snapshot"
    requested_joint = str(ctx.get("joint") or "")
    joint = requested_joint
    pose = dict(ctx.get("pose") or {})
    before = (dict(ctx.get("before") or {}) or dict(pose))
    after = dict(pose) if live_pose else (dict(ctx.get("after") or {}) or dict(pose) or dict(before))
    target = {} if live_pose else dict(ctx.get("target") or {})
    moved = False if live_pose else bool(ctx.get("moved", False))
    units = str(ctx.get("units") or "radians")

    joint_names = sorted(set(pose) | set(before) | set(after) | set(target))
    if live_pose:
        changed_joints = [
            name for name in joint_names
            if isinstance(before.get(name), (int, float)) and isinstance(after.get(name), (int, float))
        ]
        if changed_joints:
            joint = max(changed_joints, key=lambda name: abs(float(after[name]) - float(before[name])))
    delta = (after.get(joint, 0.0) - before.get(joint, 0.0)) if joint in before and joint in after else 0.0
    summary = {
        "joint": joint,
        "requested_joint": requested_joint,
        "live": live_pose,
        "mode": manual_mode,
        "torque_enabled": torque_enabled,
        "moved": moved,
        "units": units,
        "before": before.get(joint),
        "after": after.get(joint),
        "target": target.get(joint),
        "delta": delta,
        "joints": joint_names,
        "positions": dict(after or before or pose),
        "before_values": before,
        "after_values": after,
        "target_values": target,
    }

    has_motion_request = bool(joint or target) and not live_pose
    verdict = "LIVE" if live_pose else ("MOVED" if moved else ("NO CHANGE" if has_motion_request else "POSE SNAPSHOT"))
    accent = "#22c55e" if live_pose or moved else ("#ef4444" if has_motion_request else "#2e9fe6")
    muted = "#93a4b8"
    panel = "#172033"
    target_value = target.get(joint)
    current_value = after.get(joint)
    target_text = f"{target_value:.2f}" if isinstance(target_value, (int, float)) else "-"
    current_text = f"{current_value:.2f}" if isinstance(current_value, (int, float)) else "-"
    side_joint_label = "MOST CHANGED JOINT" if live_pose else "COMMANDED JOINT"
    delta_label = "CHANGE SINCE LIVE START" if live_pose else f"DELTA ({units})"
    value_label = f"CURRENT ({units})" if live_pose else "TARGET"
    value_text = current_text if live_pose else target_text

    rows = []
    for index, name in enumerate(joint_names[:8]):
        y = 196 + index * 50
        b = before.get(name)
        a = after.get(name)
        is_target = name == joint
        b_text = f"{b:.2f}" if isinstance(b, (int, float)) else "-"
        a_text = f"{a:.2f}" if isinstance(a, (int, float)) else "-"
        moved_row = isinstance(b, (int, float)) and isinstance(a, (int, float)) and abs(a - b) >= 0.01
        a_color = accent if moved_row else "#f8fafc"
        name_color = "#f8fafc" if is_target else muted
        weight_attr = ' font-weight="700"' if is_target else ""
        if is_target:
            rows.append(f'<rect x="36" y="{y - 24}" width="688" height="40" rx="10" fill="#0f1a2e" stroke="{accent}"/>')
        rows.append(
            f'<text x="60" y="{y}" fill="{name_color}" font-family="monospace" font-size="16"{weight_attr}>{_svg_text(name, 20)}</text>'
            f'<text x="430" y="{y}" text-anchor="end" fill="{muted}" font-family="monospace" font-size="16">{b_text}</text>'
            f'<text x="500" y="{y}" text-anchor="middle" fill="{muted}" font-family="Arial" font-size="14">-&gt;</text>'
            f'<text x="700" y="{y}" text-anchor="end" fill="{a_color}" font-family="monospace" font-size="16" font-weight="700">{a_text}</text>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="640" viewBox="0 0 1120 640">
<rect width="1120" height="640" rx="28" fill="#0b1020"/>
<rect x="24" y="24" width="1072" height="86" rx="18" fill="{panel}" stroke="#2e9fe6" stroke-width="2"/>
<circle cx="68" cy="67" r="18" fill="#2e9fe6"/><circle cx="68" cy="67" r="8" fill="#0b1020"/>
<text x="104" y="58" fill="#f8fafc" font-family="Arial,sans-serif" font-size="26" font-weight="700">{'LIVE MOTION DASHBOARD' if live_pose else 'MOTION RESULT · SNAPSHOT'}</text>
<text x="104" y="86" fill="{muted}" font-family="Arial,sans-serif" font-size="15">{('continuously updated · ' + ('HOLDING · TORQUE ON' if torque_enabled else 'RELEASED · TORQUE OFF')) if live_pose else 'one-time before vs after result'} ({_svg_text(units, 12)})</text>
<rect x="900" y="40" width="170" height="52" rx="26" fill="{accent}"/>
<text x="985" y="74" text-anchor="middle" fill="#ffffff" font-family="Arial,sans-serif" font-size="22" font-weight="800">{verdict}</text>

<text x="60" y="158" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">JOINT</text>
<text x="430" y="158" text-anchor="end" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">BEFORE</text>
<text x="700" y="158" text-anchor="end" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">{'CURRENT' if live_pose else 'AFTER'}</text>
{''.join(rows)}

<rect x="760" y="150" width="324" height="440" rx="16" fill="{panel}"/>
<text x="784" y="190" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">{_svg_text(side_joint_label, 28)}</text>
<text x="784" y="232" fill="#f8fafc" font-family="Arial,sans-serif" font-size="24" font-weight="800">{_svg_text(joint or "-", 18)}</text>
<text x="784" y="300" fill="{muted}" font-family="Arial,sans-serif" font-size="13">{_svg_text(delta_label, 30)}</text>
<text x="784" y="346" fill="{accent}" font-family="Arial,sans-serif" font-size="42" font-weight="800">{delta:+.2f}</text>
<text x="784" y="408" fill="{muted}" font-family="Arial,sans-serif" font-size="13">{_svg_text(value_label, 24)}</text>
<text x="784" y="448" fill="#f8fafc" font-family="monospace" font-size="22">{value_text}</text>
</svg>"""
    return {"dashboard": _svg_data(svg), "live": live_pose, "summary": summary}
