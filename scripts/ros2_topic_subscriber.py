"""Managed, named ROS 2 topic subscriber used by Blacknode.

Each received message is written as one JSON line so the editor runtime can
show structured live data while the ROS node keeps spinning.
"""
from __future__ import annotations

import argparse
import json
import math
from collections.abc import Mapping
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy import qos as rclpy_qos
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


def _json_default(value: Any) -> str:
    if isinstance(value, (bytes, bytearray, memoryview)):
        return bytes(value).hex()
    return str(value)


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="Subscribe to one ROS 2 topic")
    parser.add_argument("--node-name", required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--message-type", required=True)
    parser.add_argument("--max-messages", type=int, default=0)
    parser.add_argument("--qos", choices=["sensor_data", "reliable", "transient_local"], default="sensor_data")
    args = parser.parse_args()

    rclpy.init()
    node = Node(args.node_name)
    message_class = get_message(args.message_type)
    received = 0

    def on_message(message: Any) -> None:
        nonlocal received
        received += 1
        payload = _json_safe(message_to_ordereddict(message))
        print(
            json.dumps(
                {"message": payload},
                allow_nan=False,
                default=_json_default,
                separators=(",", ":"),
            ),
            flush=True,
        )

    qos = rclpy_qos.qos_profile_sensor_data
    if args.qos == "reliable":
        qos = rclpy_qos.QoSProfile(depth=10, reliability=rclpy_qos.ReliabilityPolicy.RELIABLE)
    elif args.qos == "transient_local":
        qos = rclpy_qos.QoSProfile(
            depth=1,
            reliability=rclpy_qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy_qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
    subscription = node.create_subscription(
        message_class,
        args.topic,
        on_message,
        qos,
    )
    try:
        while rclpy.ok() and (args.max_messages <= 0 or received < args.max_messages):
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
