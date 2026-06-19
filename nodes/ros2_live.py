"""Universal live robot-control nodes over rosbridge.

These drive **any** robot that exposes ``sensor_msgs/msg/JointState`` over a
rosbridge WebSocket: a connection/preflight doctor, a pose reader, a gated
single-joint mover, and a before/after dashboard. Topics, joint name, and units
are all inputs, so the same nodes work for any joint-based robot — robot
specifics live in templates, not in the nodes.

Motion is gated: ``ROS2RotateJoint`` does nothing unless explicitly armed, syncs
to the current pose before moving, clamps to limits when a config topic provides
them, and streams a heartbeat so a safety bridge's own timeout still applies.
"""
from __future__ import annotations

import base64
import html
import math
import sys
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

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
        lines.append("           FIX: start rosbridge_server on that port, e.g. docker start <rosbridge-container>")
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
