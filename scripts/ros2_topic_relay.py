#!/usr/bin/env python3
"""Relay one ROS 2 topic to another without interpreting the message."""
from __future__ import annotations

import argparse
import re
import signal
from typing import Any

import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def _node_name(run_id: str) -> str:
    clean = re.sub(r"[^a-zA-Z0-9_]+", "_", run_id).strip("_").lower()
    return f"blacknode_topic_relay_{clean or 'default'}"[:240]


def _qos_profile(name: str, depth: int) -> Any:
    if name == "sensor_data":
        return qos_profile_sensor_data
    return QoSProfile(depth=max(1, depth))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="default")
    parser.add_argument("--source-topic", required=True)
    parser.add_argument("--destination-topic", required=True)
    parser.add_argument("--message-type", required=True)
    parser.add_argument("--qos", choices=("reliable", "sensor_data"), default="sensor_data")
    parser.add_argument("--queue-depth", type=int, default=10)
    args = parser.parse_args()

    if args.source_topic == args.destination_topic:
        parser.error("source and destination topics must be different")

    message_class = get_message(args.message_type)
    rclpy.init()
    node = rclpy.create_node(_node_name(args.run_id))
    qos = _qos_profile(args.qos, args.queue_depth)
    publisher = node.create_publisher(message_class, args.destination_topic, qos)

    def forward(message: Any) -> None:
        publisher.publish(message)

    subscription = node.create_subscription(
        message_class,
        args.source_topic,
        forward,
        qos,
    )

    def stop(_signum: int, _frame: Any) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
