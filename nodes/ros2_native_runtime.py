"""Native ROS 2 topic access through rclpy.

This is the direct ROS 2 path: Blacknode talks to the local ROS graph with
``rclpy`` instead of going through rosbridge. Imports are lazy so the package
still loads on machines where ROS 2 is not installed or not sourced.
"""
from __future__ import annotations

import json
import math
import re
import threading
import time
from importlib import metadata
from typing import Any

JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
STRING_TYPE = "std_msgs/msg/String"

_NO_RCLPY = (
    "native rclpy is not importable in the Blacknode server Python. Start "
    "Blacknode from a ROS 2 sourced shell, for example: "
    "`source /opt/ros/jazzy/setup.bash && ./start.sh`."
)

_lock = threading.Lock()
_IMPORT_ERROR = ""


def _imports():
    global _IMPORT_ERROR
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except Exception as exc:  # noqa: BLE001 - surfaced as a structured node error
        _IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
        return None
    return {
        "rclpy": rclpy,
        "SingleThreadedExecutor": SingleThreadedExecutor,
        "JointState": JointState,
        "String": String,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
    }


def available() -> tuple[bool, str]:
    if _imports() is None:
        detail = f" ({_IMPORT_ERROR})" if _IMPORT_ERROR else ""
        return False, _NO_RCLPY + detail
    return True, ""


def rclpy_version() -> str:
    try:
        return metadata.version("rclpy")
    except Exception:  # noqa: BLE001
        return ""


def _ensure_rclpy():
    imports = _imports()
    if imports is None:
        detail = f" ({_IMPORT_ERROR})" if _IMPORT_ERROR else ""
        raise RuntimeError(_NO_RCLPY + detail)
    rclpy = imports["rclpy"]
    with _lock:
        if not rclpy.ok():
            rclpy.init(args=None)
    return imports


def _safe_node_name(value: str, fallback: str) -> str:
    candidate = re.sub(r"[^A-Za-z0-9_]", "_", str(value or "").strip().strip("/"))
    candidate = re.sub(r"_+", "_", candidate).strip("_")
    if not candidate:
        candidate = fallback
    if candidate[0].isdigit():
        candidate = f"blacknode_{candidate}"
    return candidate


def _create_rclpy_node(rclpy: Any, node_name: str):
    """Create an internal node without unused parameter/introspection services."""
    options = {
        "enable_rosout": False,
        "start_parameter_services": False,
        "enable_type_description_service": False,
    }
    try:
        return rclpy.create_node(node_name, **options)
    except TypeError:
        # ``enable_type_description_service`` was added after older supported
        # ROS 2 releases. Preserve compatibility while still disabling the
        # parameter services and rosout where those options are available.
        options.pop("enable_type_description_service")
        try:
            return rclpy.create_node(node_name, **options)
        except TypeError:
            return rclpy.create_node(node_name)


def _create_node(name: str):
    imports = _ensure_rclpy()
    node_name = f"{name}_{int(time.time() * 1000)}"
    return imports, _create_rclpy_node(imports["rclpy"], node_name)


def topic_names_and_types(timeout: float = 1.0) -> list[tuple[str, list[str]]]:
    imports, node = _create_node("blacknode_native_topics")
    rclpy = imports["rclpy"]
    try:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        return [(str(name), [str(kind) for kind in kinds]) for name, kinds in node.get_topic_names_and_types()]
    finally:
        node.destroy_node()


def _read_once(topic: str, message_cls: Any, timeout: float, qos_profiles: list[Any] | None = None):
    imports, node = _create_node("blacknode_native_read")
    rclpy = imports["rclpy"]
    box: dict[str, Any] = {}

    def on_message(message: Any) -> None:
        box["message"] = message

    try:
        profiles = qos_profiles or [10]
        subscriptions = []
        for qos in profiles:
            subscriptions.append(node.create_subscription(message_cls, topic, on_message, qos))
        deadline = time.monotonic() + max(0.0, timeout)
        while "message" not in box and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        for sub in subscriptions:
            node.destroy_subscription(sub)
        return box.get("message")
    finally:
        node.destroy_node()


def read_pose(topic: str, timeout: float = 10.0) -> dict[str, float] | None:
    imports = _ensure_rclpy()
    message = _read_once(topic, imports["JointState"], timeout)
    if message is None:
        return None
    names = list(getattr(message, "name", []) or [])
    positions = list(getattr(message, "position", []) or [])
    pose = {
        str(name): float(value)
        for name, value in zip(names, positions)
        if isinstance(value, (int, float)) and math.isfinite(value)
    }
    return pose or None


def read_config(topic: str, timeout: float = 10.0) -> dict[str, Any] | None:
    if not topic:
        return None
    imports = _ensure_rclpy()
    QoSProfile = imports["QoSProfile"]
    ReliabilityPolicy = imports["ReliabilityPolicy"]
    DurabilityPolicy = imports["DurabilityPolicy"]
    qos_profiles = [
        10,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    ]
    message = _read_once(topic, imports["String"], timeout, qos_profiles=qos_profiles)
    if message is None:
        return None
    try:
        return json.loads(getattr(message, "data", "") or "")
    except (TypeError, ValueError):
        return None


def publish_string(topic: str, value: str, timeout: float = 2.0) -> dict[str, Any]:
    imports, node = _create_node("blacknode_native_string_command")
    rclpy = imports["rclpy"]
    publisher = node.create_publisher(imports["String"], topic, 10)
    try:
        deadline = time.monotonic() + max(0.0, timeout)
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() == 0:
            return {"ok": False, "error": f"no subscribers on {topic}"}
        message = imports["String"]()
        message.data = str(value)
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.05)
        return {"ok": True}
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    finally:
        node.destroy_publisher(publisher)
        node.destroy_node()
def limits_radians(config: dict[str, Any] | None) -> dict[str, tuple[float, float]]:
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


def _joint_command_message(JointState: Any, node: Any, names: list[str], positions_radians: dict[str, float]):
    message = JointState()
    message.header.stamp = node.get_clock().now().to_msg()
    message.name = list(names)
    message.position = [float(positions_radians[name]) for name in names]
    message.velocity = []
    message.effort = []
    return message


def stream_motion(
    command_topic: str,
    names: list[str],
    start_radians: dict[str, float],
    target_radians: dict[str, float],
    *,
    ramp_seconds: float,
    hold_seconds: float,
    rate_hz: float,
    timeout: float = 10.0,
    alphas: list[float] | None = None,
) -> dict[str, Any]:
    imports, node = _create_node("blacknode_native_command")
    rclpy = imports["rclpy"]
    JointState = imports["JointState"]
    publisher = node.create_publisher(JointState, command_topic, 10)
    sent = 0
    try:
        subscriber_deadline = time.monotonic() + min(max(0.0, timeout), 1.0)
        while publisher.get_subscription_count() == 0 and time.monotonic() < subscriber_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() == 0:
            return {"ok": False, "sent": 0, "error": f"no subscribers on {command_topic}"}

        rate = max(1.0, float(rate_hz))
        period = 1.0 / rate
        hold_frames = max(0, int(max(0.0, hold_seconds) * rate))
        if alphas is None:
            ramp_frames = max(1, int(max(0.0, ramp_seconds) * rate))
            ramp_alphas = [frame / ramp_frames for frame in range(ramp_frames + 1)]
        else:
            ramp_alphas = [float(alpha) for alpha in alphas]
            if (
                len(ramp_alphas) < 2
                or not all(math.isfinite(alpha) for alpha in ramp_alphas)
                or abs(ramp_alphas[0]) > 1e-9
                or abs(ramp_alphas[-1] - 1.0) > 1e-9
                or any(left > right for left, right in zip(ramp_alphas, ramp_alphas[1:]))
                or any(alpha < 0.0 or alpha > 1.0 for alpha in ramp_alphas)
            ):
                return {"ok": False, "sent": 0, "error": "invalid normalized motion-profile samples"}
        for alpha in ramp_alphas + [1.0] * hold_frames:
            pose = {
                name: start_radians[name] + (target_radians[name] - start_radians[name]) * alpha
                for name in names
            }
            publisher.publish(_joint_command_message(JointState, node, names, pose))
            sent += 1
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(period)
        return {"ok": True, "sent": sent}
    except Exception as exc:  # never break the graph
        return {"ok": False, "sent": sent, "error": f"{type(exc).__name__}: {exc}"}
    finally:
        node.destroy_publisher(publisher)
        node.destroy_node()


class NativeJointStream:
    """Persistent native JointState subscription and command publisher."""

    def __init__(
        self,
        state_topic: str,
        command_topic: str,
        config_topic: str = "",
        *,
        timeout: float = 10.0,
        node_name: str = "",
    ):
        del timeout  # Kept for transport-compatible construction.
        imports = _ensure_rclpy()
        self._rclpy = imports["rclpy"]
        self._node = _create_rclpy_node(
            self._rclpy,
            _safe_node_name(
                node_name,
                f"blacknode_native_joint_stream_{int(time.time() * 1000)}",
            ),
        )
        self._executor = imports["SingleThreadedExecutor"]()
        self._executor.add_node(self._node)
        self._JointState = imports["JointState"]
        self._lock = threading.Condition()
        self._pose: dict[str, float] = {}
        self._config: dict[str, Any] = {}
        self._pose_at = 0.0
        self._closed = threading.Event()
        self._resources_closed = False
        self._state_sub = self._node.create_subscription(
            self._JointState,
            state_topic,
            self._on_state,
            10,
        )
        self._config_sub = None
        if config_topic:
            qos = imports["QoSProfile"](
                depth=1,
                reliability=imports["ReliabilityPolicy"].RELIABLE,
                durability=imports["DurabilityPolicy"].TRANSIENT_LOCAL,
            )
            self._config_sub = self._node.create_subscription(
                imports["String"],
                config_topic,
                self._on_config,
                qos,
            )
        self._command_pub = (
            self._node.create_publisher(self._JointState, command_topic, 10)
            if command_topic
            else None
        )
        self._thread = threading.Thread(
            target=self._spin,
            name=f"blacknode-native-joints-{id(self):x}",
            daemon=True,
        )
        self._thread.start()

    def _on_state(self, message: Any) -> None:
        names = list(getattr(message, "name", []) or [])
        positions = list(getattr(message, "position", []) or [])
        pose = {
            str(name): float(value)
            for name, value in zip(names, positions)
            if isinstance(value, (int, float)) and math.isfinite(value)
        }
        if not pose:
            return
        with self._lock:
            self._pose = pose
            self._pose_at = time.monotonic()
            self._lock.notify_all()

    def _on_config(self, message: Any) -> None:
        try:
            config = json.loads(getattr(message, "data", "") or "")
        except (TypeError, ValueError):
            return
        if not isinstance(config, dict):
            return
        with self._lock:
            self._config = config
            self._lock.notify_all()

    def _spin(self) -> None:
        while not self._closed.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception:
                self._closed.set()
                return

    def snapshot(self) -> tuple[dict[str, float], dict[str, Any], float]:
        with self._lock:
            age = (
                max(0.0, time.monotonic() - self._pose_at)
                if self._pose_at
                else float("inf")
            )
            return dict(self._pose), dict(self._config), age

    def wait_for_pose(self, timeout: float) -> dict[str, float]:
        deadline = time.monotonic() + max(0.0, timeout)
        with self._lock:
            while not self._pose and not self._closed.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._lock.wait(remaining)
            return dict(self._pose)

    def wait_for_config(self, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.0, timeout)
        with self._lock:
            while not self._config and not self._closed.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._lock.wait(remaining)
            return dict(self._config)

    def seed_config(self, config: dict[str, Any]) -> None:
        with self._lock:
            self._config = dict(config)
            self._lock.notify_all()

    def publish(self, positions_radians: dict[str, float]) -> None:
        if self._command_pub is None:
            raise RuntimeError("joint stream is read-only")
        names = list(positions_radians)
        self._command_pub.publish(
            _joint_command_message(
                self._JointState,
                self._node,
                names,
                positions_radians,
            )
        )

    def close(self) -> None:
        if self._resources_closed:
            return
        self._resources_closed = True
        self._closed.set()
        with self._lock:
            self._lock.notify_all()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        try:
            self._executor.remove_node(self._node)
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        for entity, destroy in (
            (self._state_sub, self._node.destroy_subscription),
            (self._config_sub, self._node.destroy_subscription),
            (self._command_pub, self._node.destroy_publisher),
        ):
            if entity is not None:
                try:
                    destroy(entity)
                except Exception:
                    pass
        self._node.destroy_node()


class NativeStringSubscription:
    """Persistent native std_msgs/String subscription with explicit cleanup."""

    def __init__(self, topic: str, callback: Any, *, node_name: str = ""):
        imports = _ensure_rclpy()
        self._rclpy = imports["rclpy"]
        self._callback = callback
        self._node = _create_rclpy_node(
            self._rclpy,
            _safe_node_name(
                node_name,
                f"blacknode_native_string_{time.time_ns()}_{id(self):x}",
            ),
        )
        self._executor = imports["SingleThreadedExecutor"]()
        self._executor.add_node(self._node)
        self._closed = threading.Event()
        self._subscription = self._node.create_subscription(
            imports["String"],
            topic,
            self._on_message,
            10,
        )
        self._thread = threading.Thread(
            target=self._spin,
            name=f"blacknode-native-string-{id(self):x}",
            daemon=True,
        )
        self._thread.start()

    def _on_message(self, message: Any) -> None:
        try:
            self._callback(str(getattr(message, "data", "") or ""))
        except Exception:
            # A control callback must not terminate the ROS executor thread.
            return

    def _spin(self) -> None:
        while not self._closed.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception:
                self._closed.set()
                return

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        try:
            self._executor.remove_node(self._node)
            self._executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self._node.destroy_subscription(self._subscription)
        except Exception:
            pass
        self._node.destroy_node()


def acquire_string_subscription(
    topic: str,
    callback: Any,
    *,
    node_name: str = "",
) -> NativeStringSubscription:
    return NativeStringSubscription(topic, callback, node_name=node_name)


def release_string_subscription(
    session: NativeStringSubscription | None,
) -> None:
    if session is not None:
        session.close()


def acquire_joint_stream(
    state_topic: str,
    command_topic: str,
    config_topic: str = "",
    *,
    timeout: float = 10.0,
    node_name: str = "",
) -> NativeJointStream:
    return NativeJointStream(
        state_topic,
        command_topic,
        config_topic,
        timeout=timeout,
        node_name=node_name,
    )


def release_joint_stream(
    session: NativeJointStream | None,
    *,
    discard: bool = False,
) -> None:
    del discard  # Native sessions are process-local and always close directly.
    if session is not None:
        session.close()
