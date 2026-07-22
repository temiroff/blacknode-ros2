"""Generic rosbridge topic I/O (JSON in, JSON out).

``ROS2TopicPublish``/``ROS2TopicEcho`` go through a local ros2 CLI or Docker
DDS domain. These two instead talk straight to a remote robot's rosbridge
WebSocket, so an editor on Windows can read and write any topic on the robot —
including robot-specific message types this machine has never built (the type
only has to exist on the robot's side of the bridge).
"""
from __future__ import annotations

import json
import time

from blacknode.node import Any as AnyPort
from blacknode.node import Dict, Bool, Float, Int, Text, node

from . import rosbridge_runtime as rb

_CATEGORY = "ROS 2"


@node(
    name="ROS2BridgePublish", component="rosbridge",
    category=_CATEGORY,
    description=(
        "Publish a JSON payload to any topic over rosbridge — works with robot-specific message "
        "types (e.g. a buzzer or LED state) as long as the robot knows them."
    ),
    inputs={
        "trigger": AnyPort,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "topic": Text(default=""),
        "msg_type": Text(default="std_msgs/msg/String"),
        "payload": Text(default='{"data": "hello from Blacknode"}'),
        "repeat": Int(default=1),
        "timeout": Float(default=5.0),
    },
    outputs={"published": Bool, "report": Text},
)
def ros2_bridge_publish(ctx: dict) -> dict:
    topic_name = str(ctx.get("topic") or "").strip()
    if not topic_name:
        return {"published": False, "report": "bridge publish FAILED: set topic"}
    msg_type = str(ctx.get("msg_type") or "std_msgs/msg/String").strip()
    try:
        payload = json.loads(str(ctx.get("payload") or ""))
    except (TypeError, ValueError) as exc:
        return {"published": False, "report": f"bridge publish FAILED: payload is not valid JSON ({exc})"}
    if not isinstance(payload, dict):
        return {"published": False, "report": "bridge publish FAILED: payload must be a JSON object"}

    host = str(ctx.get("host") or "127.0.0.1").strip()
    port = int(ctx.get("port") or 9090)
    repeat = min(10, max(1, int(ctx.get("repeat") or 1)))
    timeout = float(ctx.get("timeout") or 5.0)

    ok, err = rb.available()
    if not ok:
        return {"published": False, "report": f"bridge publish FAILED: {err}"}
    try:
        ros = rb.get_connection(host, port, timeout)
        publisher = rb.roslibpy.Topic(ros, topic_name, msg_type)
        publisher.advertise()
        try:
            message = rb.roslibpy.Message(payload)
            time.sleep(0.15)  # rosbridge advertisement is asynchronous
            for index in range(repeat):
                publisher.publish(message)
                if index < repeat - 1:
                    time.sleep(0.08)
            time.sleep(0.1)
        finally:
            publisher.unadvertise()
    except Exception as exc:
        return {"published": False, "report": f"bridge publish FAILED: {type(exc).__name__}: {exc}"}
    return {"published": True, "report": f"published {repeat}x {msg_type} to {topic_name} via ws://{host}:{port}"}


@node(
    name="ROS2BridgeEcho", component="rosbridge",
    category=_CATEGORY,
    description="Read one message from any topic over rosbridge and return it as a JSON dict.",
    inputs={
        "trigger": AnyPort,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "topic": Text(default=""),
        "msg_type": Text(default="std_msgs/msg/String"),
        "timeout": Float(default=5.0),
    },
    outputs={"message": Dict, "report": Text},
)
def ros2_bridge_echo(ctx: dict) -> dict:
    topic_name = str(ctx.get("topic") or "").strip()
    if not topic_name:
        return {"message": {}, "report": "bridge echo FAILED: set topic"}
    msg_type = str(ctx.get("msg_type") or "std_msgs/msg/String").strip()
    host = str(ctx.get("host") or "127.0.0.1").strip()
    port = int(ctx.get("port") or 9090)
    timeout = max(0.5, float(ctx.get("timeout") or 5.0))

    ok, err = rb.available()
    if not ok:
        return {"message": {}, "report": f"bridge echo FAILED: {err}"}
    try:
        ros = rb.get_connection(host, port, timeout)
    except Exception as exc:
        return {"message": {}, "report": f"bridge echo FAILED: {type(exc).__name__}: {exc}"}
    message = rb._read_once(ros, topic_name, msg_type, timeout)
    if message is None:
        return {"message": {}, "report": f"bridge echo FAILED: no message on {topic_name} within {timeout:g} s"}
    return {"message": dict(message), "report": f"received {msg_type} from {topic_name} via ws://{host}:{port}"}
