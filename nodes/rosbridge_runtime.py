"""Talk to any robot over a rosbridge WebSocket (roslibpy).

This is the universal command path for robots that expose their joints as
``sensor_msgs/msg/JointState`` over rosbridge (``ws://host:port``) — read the
current pose from a state topic, stream position commands to a command topic.
Positions on the wire are radians (ROS convention); unit conversion for humans
(degrees) is the node layer's job.

roslibpy runs a Twisted reactor that cannot be restarted once stopped, so we
keep one long-lived connection per ``host:port`` for the life of the server
process and never terminate it mid-session. Everything returns structured
results except :func:`get_connection`, which raises so callers can report a
clear connection error.
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

try:  # optional dependency — nodes degrade to a structured error without it
    import roslibpy

    _IMPORT_ERROR = ""
except Exception as exc:  # pragma: no cover - exercised only without roslibpy
    roslibpy = None  # type: ignore[assignment]
    _IMPORT_ERROR = f"{type(exc).__name__}: {exc}"

DEFAULT_STATE_TOPIC = "/joint_states"
DEFAULT_COMMAND_TOPIC = "/joint_commands"
JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
STRING_TYPE = "std_msgs/msg/String"

_NO_ROSLIBPY = (
    "roslibpy is not installed. Press 'Install prerequisites' in the Packages tab, "
    "run `blacknode packages setup blacknode-ros2`, or `pip install roslibpy`."
)

_lock = threading.Lock()
_connections: dict[tuple[str, int], Any] = {}
_joint_stream_lock = threading.Lock()
_joint_streams: dict[tuple[str, int, str, str, str], "JointStreamSession"] = {}


class JointStreamSession:
    """Persistent JointState subscription and command publisher.

    Continuous controllers use one of these sessions instead of subscribing,
    advertising, and tearing both entities down on every control tick.
    """

    def __init__(
        self,
        host: str,
        port: int,
        state_topic: str,
        command_topic: str,
        config_topic: str,
        timeout: float,
    ) -> None:
        self.key = (host, int(port), state_topic, command_topic, config_topic)
        self.ros = get_connection(host, port, timeout)
        self._data_lock = threading.Lock()
        self._pose_event = threading.Event()
        self._config_event = threading.Event()
        self._pose: dict[str, float] = {}
        self._config: dict[str, Any] = {}
        self._pose_updated_at = 0.0
        self._closed = False
        self._users = 0
        self._state_sub = roslibpy.Topic(self.ros, state_topic, JOINT_STATE_TYPE)
        self._command_pub = roslibpy.Topic(self.ros, command_topic, JOINT_STATE_TYPE)
        self._config_sub = roslibpy.Topic(self.ros, config_topic, STRING_TYPE) if config_topic else None
        self._state_sub.subscribe(self._on_state)
        if self._config_sub is not None:
            self._config_sub.subscribe(self._on_config)
        self._command_pub.advertise()

    def _on_state(self, message: dict) -> None:
        names = message.get("name") or []
        positions = message.get("position") or []
        pose = {
            str(name): float(value)
            for name, value in zip(names, positions)
            if isinstance(value, (int, float)) and math.isfinite(value)
        }
        if not pose:
            return
        with self._data_lock:
            self._pose = pose
            self._pose_updated_at = time.monotonic()
        self._pose_event.set()

    def _on_config(self, message: dict) -> None:
        try:
            config = json.loads(message.get("data") or "")
        except (TypeError, ValueError):
            return
        if isinstance(config, dict):
            with self._data_lock:
                self._config = config
            self._config_event.set()

    def wait_for_pose(self, timeout: float) -> dict[str, float]:
        self._pose_event.wait(max(0.0, timeout))
        with self._data_lock:
            return dict(self._pose)

    def snapshot(self) -> tuple[dict[str, float], dict[str, Any], float]:
        with self._data_lock:
            age = time.monotonic() - self._pose_updated_at if self._pose_updated_at else float("inf")
            return dict(self._pose), dict(self._config), age

    def wait_for_config(self, timeout: float) -> dict[str, Any]:
        self._config_event.wait(max(0.0, timeout))
        with self._data_lock:
            return dict(self._config)

    def publish(self, positions_radians: dict[str, float]) -> None:
        if self._closed or not self.ros.is_connected:
            raise RuntimeError("rosbridge joint stream is disconnected")
        self._command_pub.publish(_joint_command_message(list(positions_radians), positions_radians))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for topic, operation in (
            (self._state_sub, "unsubscribe"),
            (self._config_sub, "unsubscribe"),
            (self._command_pub, "unadvertise"),
        ):
            if topic is None:
                continue
            try:
                getattr(topic, operation)()
            except Exception:
                pass


def acquire_joint_stream(
    host: str,
    port: int,
    state_topic: str,
    command_topic: str,
    config_topic: str = "",
    timeout: float = 10.0,
) -> JointStreamSession:
    key = (host, int(port), state_topic, command_topic, config_topic)
    with _joint_stream_lock:
        session = _joint_streams.get(key)
        if session is None or session._closed or not session.ros.is_connected:
            if session is not None:
                session.close()
            session = JointStreamSession(*key, timeout)
            _joint_streams[key] = session
        session._users += 1
        return session


def release_joint_stream(session: JointStreamSession | None) -> None:
    if session is None:
        return
    with _joint_stream_lock:
        session._users = max(0, session._users - 1)
        if session._users == 0:
            _joint_streams.pop(session.key, None)
            session.close()


def close_joint_streams() -> int:
    with _joint_stream_lock:
        sessions = list(_joint_streams.values())
        _joint_streams.clear()
    for session in sessions:
        session.close()
    return len(sessions)


def available() -> tuple[bool, str]:
    """(True, "") when roslibpy is importable, else (False, reason)."""
    if roslibpy is None:
        return False, f"{_NO_ROSLIBPY} ({_IMPORT_ERROR})"
    return True, ""


def roslibpy_version() -> str:
    return getattr(roslibpy, "__version__", "") if roslibpy is not None else ""


def get_connection(host: str, port: int, timeout: float = 10.0):
    """Return a live, cached ``roslibpy.Ros`` for host:port, connecting if needed.

    Raises ``RuntimeError`` if roslibpy is missing or the connection cannot be
    established within ``timeout`` seconds.
    """
    ok, err = available()
    if not ok:
        raise RuntimeError(err)
    key = (host, int(port))
    with _lock:
        ros = _connections.get(key)
        if ros is not None and ros.is_connected:
            return ros
        if ros is not None and not ros.is_connected:
            try:
                ros.run(timeout=timeout)  # reuse the shared reactor thread
            except Exception:
                ros = None
        if ros is None:
            ros = roslibpy.Ros(host=host, port=int(port))
            ros.run(timeout=timeout)
            _connections[key] = ros
        if not ros.is_connected:
            raise RuntimeError(f"could not connect to rosbridge at ws://{host}:{port}")
        return ros


def _read_once(ros, topic_name: str, message_type: str, timeout: float):
    """Subscribe, capture the first message, unsubscribe. Returns dict or None."""
    topic = roslibpy.Topic(ros, topic_name, message_type)
    box: dict[str, Any] = {}
    event = threading.Event()

    def on_message(message: dict) -> None:
        box["message"] = message
        event.set()

    topic.subscribe(on_message)
    try:
        got = event.wait(timeout)
    finally:
        try:
            topic.unsubscribe()
        except Exception:
            pass
    return box.get("message") if got else None


def read_pose(host: str, port: int, topic: str, timeout: float = 10.0) -> dict[str, float] | None:
    """Read one JointState message → ``{joint: radians}`` for every named joint.

    Returns None if no message arrives within ``timeout``.
    """
    ros = get_connection(host, port, timeout)
    message = _read_once(ros, topic, JOINT_STATE_TYPE, timeout)
    if message is None:
        return None
    names = message.get("name") or []
    positions = message.get("position") or []
    pose = {
        str(name): float(value)
        for name, value in zip(names, positions)
        if isinstance(value, (int, float)) and math.isfinite(value)
    }
    return pose or None


def read_config(host: str, port: int, topic: str, timeout: float = 10.0) -> dict[str, Any] | None:
    """Read a latched ``std_msgs/String`` JSON config topic (None if absent)."""
    if not topic:
        return None
    ros = get_connection(host, port, timeout)
    message = _read_once(ros, topic, STRING_TYPE, timeout)
    if message is None:
        return None
    try:
        return json.loads(message.get("data") or "")
    except (TypeError, ValueError):
        return None


def limits_radians(config: dict[str, Any] | None) -> dict[str, tuple[float, float]]:
    """Per-joint (lower, upper) bounds in radians from a config dict, if present.

    Expects ``config["joints"][name] = {"lower": rad, "upper": rad}`` — the shape
    a rosbridge robot bridge can latch on its config topic.
    """
    limits: dict[str, tuple[float, float]] = {}
    joints = (config or {}).get("joints") or {}
    if not isinstance(joints, dict):
        return limits
    for name, spec in joints.items():
        if not isinstance(spec, dict):
            continue
        lower, upper = spec.get("lower"), spec.get("upper")
        if isinstance(lower, (int, float)) and isinstance(upper, (int, float)):
            limits[str(name)] = (min(float(lower), float(upper)), max(float(lower), float(upper)))
    return limits


def _joint_command_message(names: list[str], positions_radians: dict[str, float]):
    now = time.time()
    return roslibpy.Message(
        {
            "header": {
                "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1_000_000_000)},
                "frame_id": "",
            },
            "name": list(names),
            "position": [float(positions_radians[name]) for name in names],
            "velocity": [],
            "effort": [],
        }
    )


def stream_motion(
    host: str,
    port: int,
    command_topic: str,
    names: list[str],
    start_radians: dict[str, float],
    target_radians: dict[str, float],
    *,
    ramp_seconds: float,
    hold_seconds: float,
    rate_hz: float,
    timeout: float = 10.0,
) -> dict[str, Any]:
    """Stream a synchronized command from ``start`` to ``target`` and hold.

    The first frame equals ``start`` exactly so a safety bridge can arm torque
    without jumping. Subsequent frames ramp linearly to ``target`` over
    ``ramp_seconds`` and then hold for ``hold_seconds`` to keep a command
    heartbeat alive. All values are radians. Returns ``{ok, sent, error?}``.
    """
    ros = get_connection(host, port, timeout)
    rate = max(1.0, float(rate_hz))
    period = 1.0 / rate
    ramp_frames = max(1, int(max(0.0, ramp_seconds) * rate))
    hold_frames = max(0, int(max(0.0, hold_seconds) * rate))
    topic = roslibpy.Topic(ros, command_topic, JOINT_STATE_TYPE)
    topic.advertise()
    sent = 0
    try:
        for frame in range(ramp_frames + hold_frames + 1):
            if not ros.is_connected:
                return {"ok": False, "sent": sent, "error": "rosbridge disconnected mid-stream"}
            alpha = min(1.0, frame / ramp_frames)
            pose = {
                name: start_radians[name] + (target_radians[name] - start_radians[name]) * alpha
                for name in names
            }
            topic.publish(_joint_command_message(names, pose))
            sent += 1
            time.sleep(period)
        return {"ok": True, "sent": sent}
    except Exception as exc:  # never break the graph
        return {"ok": False, "sent": sent, "error": f"{type(exc).__name__}: {exc}"}
    finally:
        try:
            topic.unadvertise()
        except Exception:
            pass
