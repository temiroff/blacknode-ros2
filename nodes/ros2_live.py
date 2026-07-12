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
import urllib.request
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

from . import ros2_native_runtime as nr
from . import rosbridge_runtime as rb

_CATEGORY = "ROS 2"


def _svg_text(value: Any, limit: int = 90) -> str:
    text = " ".join(str(value or "").split())
    if len(text) > limit:
        text = text[: limit - 3] + "..."
    return html.escape(text)


def _svg_data(svg: str) -> str:
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


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


def _detection_center_x(detection: dict[str, Any]) -> float | None:
    center = detection.get("center")
    if isinstance(center, dict):
        value = _finite_float(center.get("x"))
        if value is not None:
            return value
    return _finite_float(detection.get("center_x"))


def _detection_width(detection: dict[str, Any], fallback: int) -> float:
    for key in ("frame_width", "image_width", "width"):
        value = _finite_float(detection.get(key))
        if value and value > 0:
            return value
    for key in ("frame", "image", "metadata"):
        nested = detection.get(key)
        if isinstance(nested, dict):
            for width_key in ("width", "frame_width", "image_width"):
                value = _finite_float(nested.get(width_key))
                if value and value > 0:
                    return value
    return max(1.0, float(fallback or 1))


def _read_detection_url(url: str, timeout: float) -> tuple[dict[str, Any], str]:
    if not url:
        return {}, ""
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "BlacknodeROS2FollowDetection/0.1"})
        with urllib.request.urlopen(req, timeout=max(0.2, timeout)) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {}, f"{type(exc).__name__}: {exc}"
    if not isinstance(payload, dict):
        return {}, "detection URL did not return a JSON object"
    detection = payload.get("detection") if isinstance(payload.get("detection"), dict) else payload
    if isinstance(detection, dict) and "found" not in detection and "found" in payload:
        detection = {**detection, "found": bool(payload.get("found"))}
    return dict(detection), ""


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
    name="ROS2NativeRobotDiscovery",
    category=_CATEGORY,
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
    name="ROS2NativeFollowDetectionJoint",
    category=_CATEGORY,
    description="Visual-servo one joint toward a CV2 detection center through native rclpy. No rosbridge required; safe by default.",
    inputs={
        "trigger": AnyPort,
        "detection": Dict,
        "detection_url": Text(default=""),
        "robot": Dict,
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "joint": Text(default=""),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "frame_width": Int(default=640),
        "target_x": Float(default=0.5),
        "deadband": Float(default=0.08),
        "gain": Float(default=35.0),
        "max_step": Float(default=8.0),
        "invert": Bool(default=False),
        "ramp_seconds": Float(default=0.35),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "moved": Bool,
        "joint": Text,
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "error": Float,
        "command": Float,
        "report": Text,
    },
)
def ros2_native_follow_detection_joint(ctx: dict) -> dict:
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    detection = ctx.get("detection") if isinstance(ctx.get("detection"), dict) else {}
    detection_url = str(ctx.get("detection_url") or "").strip()
    if detection_url:
        fetched_detection, detection_error = _read_detection_url(detection_url, timeout=1.0)
        if fetched_detection:
            detection = fetched_detection
        elif not detection:
            detection = {"found": False, "error": detection_error, "detection_url": detection_url}
    blocked = {
        "moved": False,
        "joint": joint,
        "before": {},
        "after": {},
        "target": {},
        "error": 0.0,
        "command": 0.0,
    }
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to the actuator/joint that should follow the cube."}
    if not detection:
        return {**blocked, "report": "native follow detection: no CV2 detection payload yet."}
    if detection.get("found") is False:
        if detection.get("error"):
            return {**blocked, "report": f"native follow detection: could not read {detection_url or 'detection'} ({detection['error']})."}
        return {**blocked, "report": "native follow detection: CV2 does not currently see the target."}

    center_x = _detection_center_x(detection)
    if center_x is None:
        return {**blocked, "report": "native follow detection: detection has no center.x value."}

    width = _detection_width(detection, int(ctx.get("frame_width") or 640))
    target_x_value = _finite_float(ctx.get("target_x"))
    target_x = max(0.0, min(1.0, 0.5 if target_x_value is None else target_x_value))
    normalized_x = max(0.0, min(1.0, center_x / width))
    error = target_x - normalized_x
    if bool(ctx.get("invert", False)):
        error = -error

    deadband_value = _finite_float(ctx.get("deadband"))
    deadband = max(0.0, min(0.5, 0.08 if deadband_value is None else deadband_value))
    gain_value = _finite_float(ctx.get("gain"))
    gain = 35.0 if gain_value is None else gain_value
    command = error * gain
    max_step_value = _finite_float(ctx.get("max_step"))
    max_step = abs(8.0 if max_step_value is None else max_step_value)
    if max_step > 0:
        command = max(-max_step, min(max_step, command))

    if abs(error) <= deadband:
        return {
            **blocked,
            "error": error,
            "command": 0.0,
            "report": (
                f"native follow {joint}: target centered enough "
                f"(x={center_x:.1f}/{width:.0f}, error={error:+.3f}, deadband={deadband:g}); no command streamed."
            ),
        }

    if not bool(ctx.get("armed", False)):
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": (
                f"BLOCKED: native ROS 2 visual follow preview only. Set armed=true to move {joint}. "
                f"Cube x={center_x:.1f}/{width:.0f}, error={error:+.3f}, command={command:+.2f} {units}."
            ),
        }

    ok, err = nr.available()
    if not ok:
        return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: {err}"}

    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    ramp_seconds_value = _finite_float(ctx.get("ramp_seconds"))
    hold_seconds_value = _finite_float(ctx.get("hold_seconds"))
    rate_hz_value = _finite_float(ctx.get("rate_hz"))
    timeout_value = _finite_float(ctx.get("timeout"))
    ramp_seconds = 0.35 if ramp_seconds_value is None else ramp_seconds_value
    hold_seconds = 0.2 if hold_seconds_value is None else hold_seconds_value
    rate_hz = 30.0 if rate_hz_value is None else rate_hz_value
    timeout = 10.0 if timeout_value is None else timeout_value

    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = nr.read_config(config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: {exc}"}
        if config and "commands_allowed" in config and not bool(config.get("commands_allowed")):
            return {
                **blocked,
                "error": error,
                "command": command,
                "report": "BLOCKED: the robot driver reports it is read-only (commands_allowed=false).",
            }

    try:
        start_rad = nr.read_pose(state_topic, timeout)
    except Exception as exc:
        return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "error": error, "command": command, "report": f"native follow {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}",
        }

    command_rad = _to_radians(command, units)
    raw_target_rad = start_rad[joint] + command_rad
    limits = nr.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_rad_value = min(upper, max(lower, raw_target_rad))
    else:
        target_rad_value = raw_target_rad
    names = list(start_rad.keys())
    target_rad = dict(start_rad)
    target_rad[joint] = target_rad_value

    result = nr.stream_motion(
        command_topic, names, start_rad, target_rad,
        ramp_seconds=ramp_seconds, hold_seconds=hold_seconds, rate_hz=rate_hz, timeout=timeout,
    )
    before = {n: _from_radians(v, units) for n, v in start_rad.items()}
    target = {n: _from_radians(v, units) for n, v in target_rad.items()}
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "error": error,
            "command": command,
            "report": f"native follow {joint} FAILED: {result.get('error', 'unknown error')}",
        }

    try:
        after_rad = nr.read_pose(state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {n: _from_radians(v, units) for n, v in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    clamp_note = "" if abs(raw_target_rad - target_rad_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"
    report = (
        f"native follow {joint}: cube x={center_x:.1f}/{width:.0f}, error={error:+.3f}, "
        f"command={command:+.2f} {units}, target={target[joint]:.2f}{clamp_note}; "
        f"streamed {result.get('sent', 0)} commands at {rate_hz:g} Hz"
    )
    return {
        "moved": moved,
        "joint": joint,
        "before": before,
        "after": after,
        "target": target,
        "error": error,
        "command": command,
        "report": report,
    }


@node(
    name="ROS2RobotDiscovery",
    category=_CATEGORY,
    description="Detect a connected rosbridge robot and return a generic robot profile: topics, joints, pose, limits, and command permission.",
    inputs={
        "trigger": AnyPort,
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
    description="Read the current pose from any JointState topic over rosbridge (radians, or degrees).",
    inputs={
        "trigger": AnyPort,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "topic": Text(default="/joint_states"),
        "units": Enum(["radians", "degrees"], default="radians"),
        "timeout": Float(default=10.0),
    },
    outputs={"pose": Dict, "names": List, "report": Text},
)
def ros2_joint_state(ctx: dict) -> dict:
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
    return {"pose": pose, "names": list(pose.keys()), "report": f"{len(pose)} joints ({units}): {summary}"}


@node(
    name="ROS2RotateJoint",
    category=_CATEGORY,
    description="Move one joint on a real robot over rosbridge — only when armed. Syncs to the current pose, clamps to limits, streams a heartbeat.",
    inputs={
        "trigger": AnyPort,
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
    name="ROS2FollowDetectionJoint",
    category=_CATEGORY,
    description="Visual-servo one joint toward a CV2 detection center over rosbridge. Safe by default: only streams commands when armed.",
    inputs={
        "trigger": AnyPort,
        "detection": Dict,
        "detection_url": Text(default=""),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "joint": Text(default=""),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "frame_width": Int(default=640),
        "target_x": Float(default=0.5),
        "deadband": Float(default=0.08),
        "gain": Float(default=35.0),
        "max_step": Float(default=8.0),
        "invert": Bool(default=False),
        "ramp_seconds": Float(default=0.35),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={
        "moved": Bool,
        "joint": Text,
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "error": Float,
        "command": Float,
        "report": Text,
    },
)
def ros2_follow_detection_joint(ctx: dict) -> dict:
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    detection = ctx.get("detection") if isinstance(ctx.get("detection"), dict) else {}
    detection_url = str(ctx.get("detection_url") or "").strip()
    if detection_url:
        fetched_detection, detection_error = _read_detection_url(detection_url, timeout=1.0)
        if fetched_detection:
            detection = fetched_detection
        elif not detection:
            detection = {"found": False, "error": detection_error, "detection_url": detection_url}
    blocked = {
        "moved": False,
        "joint": joint,
        "before": {},
        "after": {},
        "target": {},
        "error": 0.0,
        "command": 0.0,
    }
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to the actuator/joint that should follow the cube."}
    if not detection:
        return {**blocked, "report": "follow detection: no CV2 detection payload yet."}
    if detection.get("found") is False:
        if detection.get("error"):
            return {**blocked, "report": f"follow detection: could not read {detection_url or 'detection'} ({detection['error']})."}
        return {**blocked, "report": "follow detection: CV2 does not currently see the target."}

    center_x = _detection_center_x(detection)
    if center_x is None:
        return {**blocked, "report": "follow detection: detection has no center.x value."}

    width = _detection_width(detection, int(ctx.get("frame_width") or 640))
    target_x_value = _finite_float(ctx.get("target_x"))
    target_x = max(0.0, min(1.0, 0.5 if target_x_value is None else target_x_value))
    normalized_x = max(0.0, min(1.0, center_x / width))
    error = target_x - normalized_x
    if bool(ctx.get("invert", False)):
        error = -error

    deadband_value = _finite_float(ctx.get("deadband"))
    deadband = max(0.0, min(0.5, 0.08 if deadband_value is None else deadband_value))
    gain_value = _finite_float(ctx.get("gain"))
    gain = 35.0 if gain_value is None else gain_value
    command = error * gain
    max_step_value = _finite_float(ctx.get("max_step"))
    max_step = abs(8.0 if max_step_value is None else max_step_value)
    if max_step > 0:
        command = max(-max_step, min(max_step, command))

    if abs(error) <= deadband:
        return {
            **blocked,
            "error": error,
            "command": 0.0,
            "report": (
                f"follow {joint}: target centered enough "
                f"(x={center_x:.1f}/{width:.0f}, error={error:+.3f}, deadband={deadband:g}); no command streamed."
            ),
        }

    if not bool(ctx.get("armed", False)):
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": (
                f"BLOCKED: visual follow preview only. Set armed=true to move {joint}. "
                f"Cube x={center_x:.1f}/{width:.0f}, error={error:+.3f}, command={command:+.2f} {units}."
            ),
        }

    ok, err = rb.available()
    if not ok:
        return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: {err}"}

    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    ramp_seconds_value = _finite_float(ctx.get("ramp_seconds"))
    hold_seconds_value = _finite_float(ctx.get("hold_seconds"))
    rate_hz_value = _finite_float(ctx.get("rate_hz"))
    timeout_value = _finite_float(ctx.get("timeout"))
    ramp_seconds = 0.35 if ramp_seconds_value is None else ramp_seconds_value
    hold_seconds = 0.2 if hold_seconds_value is None else hold_seconds_value
    rate_hz = 30.0 if rate_hz_value is None else rate_hz_value
    timeout = 10.0 if timeout_value is None else timeout_value

    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = rb.read_config(host, port, config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: {exc}"}
        if config and "commands_allowed" in config and not bool(config.get("commands_allowed")):
            return {
                **blocked,
                "error": error,
                "command": command,
                "report": "BLOCKED: the robot bridge reports it is read-only (commands_allowed=false). Relaunch it to accept commands.",
            }

    try:
        start_rad = rb.read_pose(host, port, state_topic, timeout)
    except Exception as exc:
        return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "error": error, "command": command, "report": f"follow {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {
            **blocked,
            "error": error,
            "command": command,
            "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}",
        }

    command_rad = _to_radians(command, units)
    raw_target_rad = start_rad[joint] + command_rad
    limits = rb.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_rad_value = min(upper, max(lower, raw_target_rad))
    else:
        target_rad_value = raw_target_rad
    names = list(start_rad.keys())
    target_rad = dict(start_rad)
    target_rad[joint] = target_rad_value

    result = rb.stream_motion(
        host, port, command_topic, names, start_rad, target_rad,
        ramp_seconds=ramp_seconds, hold_seconds=hold_seconds, rate_hz=rate_hz, timeout=timeout,
    )
    before = {n: _from_radians(v, units) for n, v in start_rad.items()}
    target = {n: _from_radians(v, units) for n, v in target_rad.items()}
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "error": error,
            "command": command,
            "report": f"follow {joint} FAILED: {result.get('error', 'unknown error')}",
        }

    try:
        after_rad = rb.read_pose(host, port, state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {n: _from_radians(v, units) for n, v in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    clamp_note = "" if abs(raw_target_rad - target_rad_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"
    report = (
        f"follow {joint}: cube x={center_x:.1f}/{width:.0f}, error={error:+.3f}, "
        f"command={command:+.2f} {units}, target={target[joint]:.2f}{clamp_note}; "
        f"streamed {result.get('sent', 0)} commands at {rate_hz:g} Hz"
    )
    return {
        "moved": moved,
        "joint": joint,
        "before": before,
        "after": after,
        "target": target,
        "error": error,
        "command": command,
        "report": report,
    }


@node(
    name="ROS2MotionDashboard",
    category=_CATEGORY,
    description="Render before/after joint values for any robot so the graph visibly shows it moved.",
    inputs={
        "joint": Text(default=""),
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "moved": Bool(default=False),
        "units": Text(default="radians"),
    },
    outputs={"dashboard": Image, "summary": Dict},
)
def ros2_motion_dashboard(ctx: dict) -> dict:
    joint = str(ctx.get("joint") or "")
    before = dict(ctx.get("before") or {})
    after = dict(ctx.get("after") or {})
    target = dict(ctx.get("target") or {})
    moved = bool(ctx.get("moved", False))
    units = str(ctx.get("units") or "radians")

    joint_names = sorted(set(before) | set(after))
    delta = (after.get(joint, 0.0) - before.get(joint, 0.0)) if joint in before and joint in after else 0.0
    summary = {
        "joint": joint,
        "moved": moved,
        "units": units,
        "before": before.get(joint),
        "after": after.get(joint),
        "target": target.get(joint),
        "delta": delta,
    }

    verdict = "MOVED" if moved else "NO CHANGE"
    accent = "#22c55e" if moved else "#ef4444"
    muted = "#93a4b8"
    panel = "#172033"
    target_value = target.get(joint)
    target_text = f"{target_value:.2f}" if isinstance(target_value, (int, float)) else "-"

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
<text x="104" y="58" fill="#f8fafc" font-family="Arial,sans-serif" font-size="26" font-weight="700">ROS 2 LIVE MOTION TEST</text>
<text x="104" y="86" fill="{muted}" font-family="Arial,sans-serif" font-size="15">before vs after joint values ({_svg_text(units, 12)}) over rosbridge</text>
<rect x="900" y="40" width="170" height="52" rx="26" fill="{accent}"/>
<text x="985" y="74" text-anchor="middle" fill="#ffffff" font-family="Arial,sans-serif" font-size="22" font-weight="800">{verdict}</text>

<text x="60" y="158" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">JOINT</text>
<text x="430" y="158" text-anchor="end" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">BEFORE</text>
<text x="700" y="158" text-anchor="end" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">AFTER</text>
{''.join(rows)}

<rect x="760" y="150" width="324" height="440" rx="16" fill="{panel}"/>
<text x="784" y="190" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">COMMANDED JOINT</text>
<text x="784" y="232" fill="#f8fafc" font-family="Arial,sans-serif" font-size="24" font-weight="800">{_svg_text(joint or "-", 18)}</text>
<text x="784" y="300" fill="{muted}" font-family="Arial,sans-serif" font-size="13">DELTA ({_svg_text(units, 10)})</text>
<text x="784" y="346" fill="{accent}" font-family="Arial,sans-serif" font-size="42" font-weight="800">{delta:+.2f}</text>
<text x="784" y="408" fill="{muted}" font-family="Arial,sans-serif" font-size="13">TARGET</text>
<text x="784" y="448" fill="#f8fafc" font-family="monospace" font-size="22">{target_text}</text>
</svg>"""
    return {"dashboard": _svg_data(svg), "summary": summary}
