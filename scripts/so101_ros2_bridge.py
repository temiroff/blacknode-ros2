#!/usr/bin/env python
"""Safety-gated ROS 2 bridge for a LeRobot SO-ARM101 follower arm.

The bridge publishes observations immediately. Motion commands are accepted
only when both ``--enable-motion`` was passed at launch and ``/so101/enable``
has received ``true``. A true message on ``/so101/stop`` latches a software
stop until the process is restarted.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any

JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="SO-ARM101 Feetech serial port")
    parser.add_argument("--robot-id", default="my_so101", help="LeRobot calibration id")
    parser.add_argument("--camera-index", type=int, default=-1, help="OpenCV camera index; -1 disables it")
    parser.add_argument("--camera-name", default="front")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--max-relative-target", type=float, default=5.0)
    parser.add_argument("--command-timeout", type=float, default=0.75)
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="Allow the ROS enable gate to arm motor commands",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the topic contract without importing ROS or touching hardware",
    )
    return parser


def topic_contract(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "mode": "motion-capable" if args.enable_motion else "observation-only",
        "joint_order": JOINTS,
        "command_units": "degrees for first five joints; 0-100 for gripper",
        "publishes": {
            "/so101/state": "std_msgs/msg/Float64MultiArray",
            "/joint_states": "sensor_msgs/msg/JointState",
            f"/so101/camera/{args.camera_name}/compressed": "sensor_msgs/msg/CompressedImage",
            "/so101/status": "std_msgs/msg/String",
        },
        "subscribes": {
            "/so101/command": "std_msgs/msg/Float64MultiArray",
            "/so101/enable": "std_msgs/msg/Bool",
            "/so101/stop": "std_msgs/msg/Bool",
        },
    }


def run_bridge(args: argparse.Namespace) -> int:
    try:
        import cv2
        import rclpy
        from lerobot.cameras.opencv import OpenCVCameraConfig
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        from rclpy.node import Node
        from sensor_msgs.msg import CompressedImage, JointState
        from std_msgs.msg import Bool, Float64MultiArray, String
    except ImportError as exc:
        print(
            "Missing bridge dependency. Install ROS 2 Python packages and LeRobot "
            'with its Feetech extra (`pip install -e ".[feetech]"`).\n'
            f"Import error: {exc}",
            file=sys.stderr,
        )
        return 2

    camera_configs = {}
    if args.camera_index >= 0:
        camera_configs[args.camera_name] = OpenCVCameraConfig(
            index_or_path=args.camera_index,
            width=args.width,
            height=args.height,
            fps=args.fps,
        )
    robot_config = SO101FollowerConfig(
        port=args.port,
        id=args.robot_id,
        cameras=camera_configs,
        max_relative_target=max(0.1, args.max_relative_target),
        disable_torque_on_disconnect=True,
        use_degrees=True,
    )
    robot = SO101Follower(robot_config)
    if not robot.calibration:
        print(
            f"No LeRobot calibration found for robot id {args.robot_id!r}. "
            "Run lerobot-calibrate before starting this bridge.",
            file=sys.stderr,
        )
        return 3

    class SO101Bridge(Node):
        def __init__(self) -> None:
            super().__init__("so101_lerobot_bridge")
            self.motion_capable = bool(args.enable_motion)
            self.motion_enabled = False
            self.stop_latched = False
            self.pending_command: tuple[list[float], float] | None = None

            self.state_pub = self.create_publisher(Float64MultiArray, "/so101/state", 10)
            self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.status_pub = self.create_publisher(String, "/so101/status", 10)
            self.image_pub = None
            if args.camera_index >= 0:
                self.image_pub = self.create_publisher(
                    CompressedImage,
                    f"/so101/camera/{args.camera_name}/compressed",
                    2,
                )

            self.create_subscription(
                Float64MultiArray, "/so101/command", self.on_command, 10
            )
            self.create_subscription(Bool, "/so101/enable", self.on_enable, 10)
            self.create_subscription(Bool, "/so101/stop", self.on_stop, 10)
            self.timer = self.create_timer(1.0 / max(1, args.fps), self.tick)

        def publish_status(self, message: str) -> None:
            msg = String()
            msg.data = message
            self.status_pub.publish(msg)
            self.get_logger().info(message)

        def on_enable(self, msg: Any) -> None:
            if not msg.data:
                self.motion_enabled = False
                self.publish_status("motion disabled")
                return
            if not self.motion_capable:
                self.publish_status("enable rejected: bridge launched without --enable-motion")
                return
            if self.stop_latched:
                self.publish_status("enable rejected: software stop is latched; restart bridge")
                return
            self.motion_enabled = True
            self.publish_status("motion enabled; waiting for a fresh command")

        def on_stop(self, msg: Any) -> None:
            if msg.data:
                self.stop_latched = True
                self.motion_enabled = False
                self.pending_command = None
                self.publish_status("software stop latched; restart bridge to clear")

        def on_command(self, msg: Any) -> None:
            values = [float(value) for value in msg.data]
            if len(values) != len(JOINTS):
                self.publish_status("command rejected: expected six joint values")
                return
            if not all(math.isfinite(value) for value in values):
                self.publish_status("command rejected: values must be finite")
                return
            if not 0.0 <= values[-1] <= 100.0:
                self.publish_status("command rejected: gripper must be in range 0..100")
                return
            self.pending_command = (values, time.monotonic())
            if not self.motion_enabled:
                self.publish_status("command received but motion gate is disabled")

        def publish_observation(self, observation: dict[str, Any]) -> None:
            native = [float(observation[f"{name}.pos"]) for name in JOINTS]

            state = Float64MultiArray()
            state.data = native
            self.state_pub.publish(state)

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(JOINTS)
            joint_state.position = [
                *[math.radians(value) for value in native[:5]],
                native[5] / 100.0,
            ]
            self.joint_pub.publish(joint_state)

            if self.image_pub is not None and args.camera_name in observation:
                ok, encoded = cv2.imencode(".jpg", observation[args.camera_name])
                if ok:
                    image = CompressedImage()
                    image.header.stamp = joint_state.header.stamp
                    image.format = "jpeg"
                    image.data = encoded.tobytes()
                    self.image_pub.publish(image)

        def apply_pending_command(self) -> None:
            if not self.motion_enabled or self.pending_command is None:
                return
            values, received_at = self.pending_command
            self.pending_command = None
            age = time.monotonic() - received_at
            if age > args.command_timeout:
                self.publish_status(f"command rejected: stale by {age:.2f}s")
                return
            action = {f"{name}.pos": value for name, value in zip(JOINTS, values)}
            sent = robot.send_action(action)
            self.publish_status(f"command sent: {sent}")

        def tick(self) -> None:
            try:
                self.publish_observation(robot.get_observation())
                self.apply_pending_command()
            except Exception as exc:
                self.motion_enabled = False
                self.pending_command = None
                self.publish_status(f"bridge error; motion disabled: {exc}")

    rclpy.init()
    bridge = None
    try:
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise RuntimeError(
                "motor calibration does not match the saved LeRobot calibration"
            )
        if not args.enable_motion:
            robot.bus.disable_torque()
        bridge = SO101Bridge()
        bridge.publish_status(
            "bridge ready in "
            + ("motion-capable mode; ROS enable is false" if args.enable_motion else "observation-only mode")
        )
        rclpy.spin(bridge)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"SO-ARM101 bridge failed: {exc}", file=sys.stderr)
        return 4
    finally:
        if bridge is not None:
            bridge.destroy_node()
        if robot.is_connected:
            robot.disconnect()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    args = build_parser().parse_args()
    if args.dry_run:
        print(json.dumps(topic_contract(args), indent=2))
        return 0
    return run_bridge(args)


if __name__ == "__main__":
    raise SystemExit(main())
