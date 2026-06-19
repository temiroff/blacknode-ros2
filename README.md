# blacknode-ros2

**ROS 2 nodes for [Blacknode](https://github.com/temiroff/Blacknode).**

This is a Blacknode **extension package** — it does not run on its own. It
plugs ROS 2 into the Blacknode visual workflow editor: list topics and
services, echo and publish messages, inspect interface definitions, and drive
it all from workflows or AI agents over MCP.

No ROS installation is required: if `ros2` isn't on your PATH, the package
runs everything inside a Docker helper container (`ros:jazzy`), which works on
Windows, macOS, and Linux. With a native/WSL ROS 2 install it talks to your
real ROS graph directly.

## Requirements

- The [Blacknode](https://github.com/temiroff/Blacknode) main app
- **One of:**
  - Docker (the install step pulls `ros:jazzy` automatically), or
  - a native ROS 2 installation with `ros2` on PATH

Neither installed? The nodes still load and return structured "ROS 2 not
available" results with setup instructions, so workflows stay viewable
anywhere.

## Install

From the Blacknode repo root:

```bash
blacknode packages install git@github.com:temiroff/blacknode-ros2.git
```

This clones the repo into `packages/` and pulls the `ros:jazzy` Docker image
declared in the manifest. If you cloned by hand, install the prerequisites
with:

```bash
blacknode packages setup blacknode-ros2
```

Restart Blacknode (or press **Reload** in the editor's Packages tab). The
nodes appear under the **ROS 2** palette category.

## The nodes

| Node | What it does |
|---|---|
| `ROS2SystemCheck` | Detect the backend (native / Docker / unavailable) and probe the ROS graph |
| `ROS2TopicList` | List live topics, optionally with message types |
| `ROS2TopicEcho` | Read N messages from a topic, bounded by a timeout |
| `ROS2CompressedImageSnapshot` | Capture and display one compressed ROS camera frame |
| `ROS2TopicPublish` | Publish one or more messages (YAML payload) to a topic |
| `ROS2DemoPublisher` | Start/stop a background publisher so you can demo without a robot |
| `ROS2NodeList` | List running ROS nodes |
| `ROS2ServiceList` | List live services, optionally with types |
| `ROS2InterfaceShow` | Show a message/service definition — lets AI agents compose valid payloads |
| `ROS2Command` | Escape hatch: run any `ros2 ...` subcommand and capture the output |
| `ROS2RosbridgeStatus` | Preflight a rosbridge robot connection (roslibpy, WebSocket, optional config) with exact fixes |
| `ROS2JointState` | Read any robot's current pose from a `JointState` topic (radians or degrees) |
| `ROS2RotateJoint` | Move one joint on a **real** robot — gated by `armed`, syncs to the current pose, clamps to limits, streams a heartbeat |
| `ROS2MotionDashboard` | Render before/after joint values so the graph visibly shows the robot moved |

Action nodes carry an optional `trigger` input so you can sequence them in a
graph (start the publisher → then echo).

## Templates

Loadable from the editor's Templates tab:

- **ROS 2 System Check** — quick preflight with a visible backend status output
- **ROS 2 Live Roundtrip Demo** — press the top-bar **Run** button to start a
  publisher on `/blacknode_demo`, capture a real message, and render a large
  visual dashboard with the message path, pass/fail checks, and graph metrics
- **ROS 2 Live Motion Test** — connects to your running rosbridge, reads
  the live pose, and (once you set `armed=true`) rotates one joint on the real
  robot, rendering a before/after dashboard that proves it moved

To verify it visually:

1. Start Blacknode and open the **Templates** tab.
2. Load **ROS 2 Live Roundtrip Demo**.
3. Press the green top-bar **Run** button.
4. Confirm the dashboard verdict is green **PASS**.
5. Confirm the message path shows `PUBLISHER PASS`, `/blacknode_demo`
   discovery `PASS`, and `ECHO CAPTURE PASS`.
6. Confirm the captured message card contains
   `data: Blacknode ROS 2 roundtrip works`.

The demo publisher remains active so you can recook individual nodes. To stop
it, select `ROS2DemoPublisher`, change `action` to `stop`, and cook that node,
or run:

```bash
docker exec blacknode-ros2 pkill -f "ros2 topic pub"
```

## Backend details

| Situation | Behavior |
|---|---|
| `ros2` on PATH | Commands run natively against your ROS graph |
| Docker only | A persistent helper container `blacknode-ros2` (image `ros:jazzy`) starts on first use; commands run via `docker exec` |
| Neither | Structured error with setup instructions |

Environment overrides: `BLACKNODE_ROS2_IMAGE` (default `ros:jazzy`),
`BLACKNODE_ROS2_CONTAINER` (default `blacknode-ros2`). Remove the helper
container any time with `docker rm -f blacknode-ros2` — it is recreated on
demand.

Note: the Docker backend is a self-contained ROS graph inside the container —
great for demos, learning, and agent development. To talk to robots on your
LAN, use a native/WSL ROS 2 install (DDS discovery does not cross the
Docker Desktop NAT on Windows/macOS).

## SO-ARM101 bridge

The SO-ARM101 is controlled by LeRobot over its Feetech serial bus. The
included `scripts/so101_ros2_bridge.py` wraps that API in ROS 2:

| Direction | Topic | Type |
|---|---|---|
| publish | `/so101/state` | `std_msgs/msg/Float64MultiArray` in LeRobot native units |
| publish | `/joint_states` | `sensor_msgs/msg/JointState` |
| publish | `/so101/camera/front/compressed` | `sensor_msgs/msg/CompressedImage` |
| publish | `/so101/status` | `std_msgs/msg/String` |
| subscribe | `/so101/command` | `std_msgs/msg/Float64MultiArray` |
| subscribe | `/so101/enable` | `std_msgs/msg/Bool` |
| subscribe | `/so101/stop` | `std_msgs/msg/Bool` |

Command order is `shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`, `gripper`. The first five values are degrees and
the gripper is `0..100`.

First configure and calibrate the follower using LeRobot:

```bash
lerobot-find-port
lerobot-setup-motors --robot.type=so101_follower --robot.port=COM3
lerobot-calibrate --robot.type=so101_follower --robot.port=COM3 --robot.id=my_so101
```

Inspect the bridge contract without importing ROS or touching hardware:

```bash
python packages/blacknode-ros2/scripts/so101_ros2_bridge.py --port COM3 --dry-run
```

Start observation-only mode first. It publishes joint state and camera data,
rejects motion enables, and disables motor torque after connecting:

```bash
python packages/blacknode-ros2/scripts/so101_ros2_bridge.py \
  --port COM3 --robot-id my_so101 --camera-index 0
```

Physical motion has three independent software gates:

1. Start the bridge with `--enable-motion`.
2. Publish `true` to `/so101/enable`.
3. Publish a position command to `/so101/command` (e.g. with `ROS2TopicPublish`).

A true message on `/so101/stop` latches a software stop until the bridge is
restarted. This is not a certified emergency stop; keep a physical power
cutoff within reach and clear the robot workspace before enabling motion.

## Live robot control over rosbridge (universal)

The `ROS2RosbridgeStatus`, `ROS2JointState`, `ROS2RotateJoint`, and
`ROS2MotionDashboard` nodes drive **any** robot that exposes its joints as
`sensor_msgs/msg/JointState` over a rosbridge WebSocket, using `roslibpy`.
Topics, joint name, and units are all node inputs — robot specifics live in
templates, not in the nodes. This is independent of the bundled
`scripts/so101_ros2_bridge.py` (native DDS) described above.

| Input | Default | Meaning |
|---|---|---|
| `state_topic` | `/joint_states` | `JointState` to read the current pose from |
| `command_topic` | `/joint_commands` | `JointState` to stream position commands to (radians on the wire) |
| `config_topic` | (empty) | optional latched `std_msgs/String` JSON with `commands_allowed` + per-joint `lower`/`upper` limits |
| `units` | `radians` | `radians` (ROS standard) or `degrees` for the values you type and see |

`roslibpy` is installed by **Install prerequisites** in the Packages tab,
`blacknode packages setup blacknode-ros2`, or `pip install roslibpy` **into the
Blacknode server environment**. Without it the nodes load and return a
structured "roslibpy not installed" result, and the Packages tab flags it.

The included **ROS 2 Live Motion Test** template pre-fills these inputs
(`/joint_states`, `/joint_commands`, `/joint_config`, `degrees`, the `gripper`
joint); repoint host/port/topics/joint for your robot.

**To move a real robot:**

1. Start a rosbridge server for your robot, serving `ws://127.0.0.1:9090`.
2. Start your robot's safety-gated command bridge so it accepts commands (for
   the bundled SO-ARM101 example, see the [SO-ARM101 bridge](#so-arm101-bridge)
   section below).
3. In Blacknode, load **ROS 2 Live Motion Test** and press **Run** — the
   dashboard shows the live pose with `armed=false` (no motion).
4. Set the `ROS2RotateJoint` node's `armed=true` and recook. It syncs to the
   current pose, ramps the chosen joint by `delta`, streams the command at
   `rate_hz` for `hold_seconds`, and reports the before/after angles.

Safety, layered on top of the bridge's own torque/heartbeat gates:

- `armed=false` (default) never opens a connection or sends anything.
- A read-only bridge (no `--allow-commands`) is refused with a clear message.
- The first command always equals the current pose, so the arm never jumps.
- Targets are clamped to the calibration limits reported on `/joint_config`.

Keep a physical power cutoff within reach and clear the workspace before arming.

## Development

After loading, modules are importable through Blacknode's stable alias:

```python
from blacknode.pkg.blacknode_ros2 import ros2_runtime
```

The suite in `tests/` runs automatically with `pytest` from the Blacknode repo
root. Integration tests skip cleanly without a backend; with Docker running
they exercise a real publish → echo roundtrip.

## License

Apache-2.0, same as Blacknode.
