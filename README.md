# blacknode-ros2

The package is the horizontal `ros2` **integration layer**. Native DDS is the
default robot-control path through `core`, `topics`, `services`, and
`diagnostics`. `rosbridge` and process launching are optional components for
browser, remote-workflow, and managed launch use cases. Camera, arm, base,
policy, and skill nodes are ROS 2 *adapters* that
live in the package owning that capability and declare a versioned dependency
on `blacknode-ros2/core`.

**ROS 2 integration for [Blacknode](https://github.com/temiroff/Blacknode).**

Install this Blacknode **extension package** to add ROS 2 to the visual
workflow editor: list topics and
services, echo and publish messages, inspect interface definitions, and drive
it all from workflows or AI agents over MCP.

On Linux, Blacknode prefers the native ROS 2 installation and discovers
standard `/opt/ros/<distro>/setup.bash` environments even when the runtime
starts through systemd. Joint-state, command, manual-move, and leader/follower
adapters use native `rclpy` directly. Rosbridge and Docker remain on-demand
compatibility transports for systems that use WebSocket or the `ros:jazzy`
helper image.

On Windows, if Docker is installed but Docker Desktop isn't running yet, the
first ROS 2 node that needs it launches Docker Desktop and waits for the
daemon before continuing. Starting Blacknode, opening the editor, reading
runtime status, and stopping an empty runtime do not probe or start Docker.

## Requirements

- The [Blacknode](https://github.com/temiroff/Blacknode) main app
- **One of:**
  - a native ROS 2 installation; rosbridge workflows also use the matching
    `ros-<distro>-rosbridge-server` package, or
  - Docker Desktop on Windows/macOS, or Docker Engine on Linux, for the
    on-demand `ros:jazzy` fallback

Neither installed? The nodes still load and return structured "ROS 2 not
available" results with setup instructions, so workflows stay viewable
anywhere.

## Install

From the Blacknode repo root:

```bash
blacknode packages install git@github.com:temiroff/blacknode-ros2.git
```

This clones the repo into `packages/`. If you cloned by hand, install the
Python prerequisites with:

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
| `ROS2TopicPublisher` | Publish once, start continuously, or stop a topic publisher with an optional explicit ROS node name |
| `ROS2TopicSubscriber` | Subscribe once, start continuously, or stop a named subscriber while retaining structured recent messages |
| `ROS2TopicRelay` | Continuously subscribe to one typed data topic and republish unchanged messages on another topic |
| `ROS2WorkspaceBuild` | Copy and build a local colcon workspace for the active native or Docker ROS backend |
| `ROS2PythonNode` | Start or stop a standalone Python `rclpy` node from a workspace file or inline code |
| `ROS2Launch` | Start/stop a background `ros2 launch ...` process by stable `run_id` |
| `ROS2Run` | Start/stop a background `ros2 run <package> <executable> ...` process |
| `ROS2GraphExplorer` | Capture nodes, typed topics, services, publisher/subscriber endpoints, and QoS summaries as one visual topology |
| `ROS2NodeList` | List running ROS nodes |
| `ROS2ServiceList` | List live services, optionally with types |
| `ROS2InterfaceShow` | Show a message/service definition — lets AI agents compose valid payloads |
| `ROS2PackageExecutables` | List executable commands registered by a ROS 2 package |
| `ROS2Status` | Auto-select native `rclpy` or rosbridge, ensuring the local rosbridge service when needed |
| `ROS2BridgePublish` | Publish any message type to a topic over a rosbridge WebSocket |
| `ROS2BridgeEcho` | Read messages from a topic over a rosbridge WebSocket |

Action nodes carry an optional `trigger` input so you can sequence them in a
graph (start the publisher → then echo).

`ROS2TopicPublisher` owns the complete publishing lifecycle. Set `action=once`
to send `count` messages and exit, `action=start` to publish continuously at
`rate_hz`, or `action=stop` to stop the managed publisher. `node_name` gives
the publishing node an explicit name in the ROS graph.

`ROS2TopicSubscriber` provides the matching subscription lifecycle. Set
`action=start`, choose a stable `node_name`, topic, and message type, and cook
the node once. Its `latest`, `messages`, and `received` outputs update while the
subscriber is live. Set `action=stop` to stop it, or `action=once` for one named
bounded read. `history` retains the most recent 1–100 structured messages.

`ROS2PythonNode` runs standalone `rclpy` programs as managed processes. In
`source_mode=file`, set `script_path` to a `.py` file; Docker-backed runtimes
copy it into the ROS helper container before starting it. In
`source_mode=inline`, place the program in `code`. `run_id` owns replacement,
runtime reattachment, and stopping, while `arguments` uses normal shell-style
argument splitting. The live `logs` output retains the most recent 50 stdout
and stderr lines. The Python program defines its own ROS node names, topics,
services, parameters, and timers.

`ROS2WorkspaceBuild` turns a workspace containing `src/` into a sourced ROS 2
overlay. Set `workspace_path` to an absolute path or a path relative to the
Blacknode workspace. `packages_select` optionally limits the colcon build to a
space-separated package list. Its `workspace_path` output connects directly to
the matching optional input on `ROS2Run` or `ROS2Launch`. With the Docker
backend, Blacknode copies the source tree into the managed ROS helper container,
builds it there, and sources that overlay for the run or launch process.

### Explore the live graph

`ROS2GraphExplorer` captures a read-only topology snapshot and renders it on
the workflow canvas as publisher → typed topic → subscriber paths. Its Topics,
Nodes, and Services tabs share one search field, and endpoint cards show the
available reliability and durability QoS values. **Add monitor** creates a
camera, depth, joint-state, or generic topic monitor beside the explorer based
on the selected message type. **Add subscriber** creates a persistent named
`ROS2TopicSubscriber`; **Add one-shot monitor** keeps the bounded debugging
behavior explicit.

Use `namespace` to focus one robot or subsystem. Endpoint inspection is
bounded by `max_topics` (40 by default) and runs concurrently; turn
`include_endpoints` off for a faster name-and-type inventory. The `graph`
output is a versioned dictionary containing the capture time, backend, nodes,
typed topics, services, endpoint details, warnings, and truncation state, so
the same snapshot can feed other workflow nodes or an agent.

Managed processes are scoped by stable IDs and topic groups can be inspected
as one contract. This lets a camera provider verify RGB, camera-info, depth,
and point-cloud publishers together while remaining independent from robot
motion deployments. The semantic camera and depth nodes live in
`blacknode-perception`; raw ROS execution, discovery, and image transport live
here.

## Components

Each selectable component owns its node registration path and depends on the
shared `core` runtime where needed:

| Component | Default | Provides |
|---|---:|---|
| `core` | On | Stable native ROS 2 and DDS runtime contracts |
| `topics` | On | Topic discovery, bounded publish/echo, and continuous publishing |
| `services` | On | Service discovery |
| `diagnostics` | On | Backend status, graph inspection, and interface inspection |
| `rosbridge` | Off | WebSocket topic I/O, connection status, and local server lifecycle |
| `processes` | Off | Local workspace builds and managed `ros2 run`/`ros2 launch` processes |

Templates declare the exact components they use. Enabling any feature component
resolves `core` first, and disabling a component removes its nodes from package
discovery.

## What this package deliberately does not contain

This is the horizontal **integration layer**: the ROS graph, topics, services,
processes, and the native/rosbridge transports. Capability nodes are owned by
the capability's own package and adapt it to this layer through a versioned
dependency on `blacknode-ros2/core`:

| Capability node | Lives in |
|---|---|
| `CameraROS2Provider`, `CameraROS2Subscribe`, `CameraROS2Publish`, `CameraROS2Http`, `DepthROS2Subscribe` | `blacknode-perception` → `camera/ros2` and `depth/ros2` adapters |
| `ROS2JointState`, `ROS2SetJoint`, `ROS2ManualMove`, `ROS2MotionDashboard` | `blacknode-motion` → `arm/ros2` adapter |
| `ROS2BaseMove`, `ROS2BaseStop`, `ROS2LaserScanCheck`, `ROS2OdomState` | `blacknode-motion` → `base/ros2` adapter |
| `PolicyRuntime`, `PolicySafetyGate` | `blacknode-motion` → `policy/ros2` adapter |
| `ROS2FollowDetectionJoint`, `ROS2LeaderFollower` | `blacknode-skills` → `follow/ros2` adapter |

Keeping the split this way means a second transport (Zenoh, MQTT, a direct
Python bridge) can be added later as a sibling adapter without reorganizing
any capability package.

## Policy deployment

`PolicyRuntime` consumes a `blacknode.policy-artifact`, the follower `Robot`,
and the same named `blacknode.frame-stream` cameras used to record training
episodes. Starting the runtime only begins a disarmed prediction preview.
Choose `action=arm` in a separate cook after checking live predictions and the
workspace. The first armed command synchronizes to the current joint pose.

Every later action passes through calibrated joint limits, maximum joint
velocity, maximum per-cycle step, and source-freshness checks. Optional
workspace bounds require a live `geometry_msgs/PoseStamped` topic; missing or
out-of-bounds workspace telemetry suppresses commands. `disarm`, `estop`,
`takeover`, inference faults, stale sources, normal stop, and server shutdown
request torque release through the robot driver. Support the arm before any
torque-release action because gravity may move it.

Runtime metrics and inference/command decisions are appended to
`.blacknode/policy-runs/<run_id>.jsonl` by default for replay and failure
review. Camera pixels are not copied into this log; dataset recording remains
the source of synchronized correction episodes.

## Templates

Loadable from the editor's Templates tab. Every template is self-contained:
the leading check/status node starts rosbridge on the native Linux ROS 2 graph
or selects the configured fallback transport.

One template per feature — no overlapping variants:

| Template | Feature it shows |
|---|---|
| **Publish & Subscribe Messages** | Messaging. Publishes on `/blacknode_demo`, subscribes and reads one back, lists the live graph, and draws a PASS/FAIL dashboard. |
| **Relay Any ROS 2 Data Topic** | Data routing. Continuously subscribes to a configurable source topic and republishes the same typed messages on a configurable destination topic. |
| **Run Your Own ROS 2 Package** | Process control. `ros2 launch` your own package, then confirm which topics and nodes appeared. |
| **Explore the Live ROS 2 Graph** | Topology inspection. Filters a read-only graph and maps publishers through typed topics to subscribers. |
| **Connect to a Robot Over WiFi** | Remote transport. Reaches a robot running `rosbridge_server` at `ROBOT_IP` over a WebSocket: check, read a topic, publish back. |

To verify it visually:

1. Start Blacknode and open the **Templates** tab.
2. Load **Publish & Subscribe Messages**.
3. Press the green top-bar **Run** button.
4. Confirm the dashboard verdict is green **PASS**.
5. Confirm the message path shows `PUBLISHER PASS`, `/blacknode_demo`
   discovery `PASS`, and `ECHO CAPTURE PASS`.
6. Confirm the captured message card contains
   `data: Blacknode ROS 2 roundtrip works`.

Camera and joint-motion templates ship with the packages that own those
capabilities: **Camera — Live Video** with `blacknode-perception`, and
**Move a Robot Joint** with `blacknode-motion`. Both still appear in the
same Templates tab once those packages are installed.

The topic publisher remains active so you can recook individual nodes. To stop
it, select `ROS2TopicPublisher`, change `action` to `stop`, and cook that node,
or run:

```bash
docker exec blacknode-ros2 pkill -f "ros2 topic pub"
```

`ROS2TopicRelay` is for data topics such as camera metadata, IMU, LiDAR,
battery, diagnostics, and application events. It preserves the ROS message
type and offers reliable or sensor-data QoS. Known motion destinations such as
`/cmd_vel`, joint-command, servo-command, robot-control, and trajectory topics
are rejected. Use the matching Blacknode controller for motion so explicit
arming, calibration limits, source freshness, and shutdown behavior remain
enforced.

## Backend details

| Situation | Behavior |
|---|---|
| `ros2` on PATH | Commands run natively against your ROS graph |
| Docker only | A persistent helper container `blacknode-ros2` (image `ros:jazzy`) starts on first use; commands run via `docker exec` |
| Neither | Structured error with setup instructions |

Environment overrides: `BLACKNODE_ROS2_IMAGE` (default `ros:jazzy`),
`BLACKNODE_ROS2_CONTAINER` (default `blacknode-ros2`), and
`BLACKNODE_ROS2_STREAM_PORT_RANGE` (default `39000-39049`). For native
workspaces, `./start.sh` auto-sources `/opt/ros/jazzy/setup.bash` when present
and auto-sources a workspace only when it finds exactly one
`ros2_ws/install/setup.bash`. If you have multiple ROS workspaces, source the
one you want before starting Blacknode so the overlay order is explicit:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/ros2_ws/install/setup.bash
./start.sh
```

Remove the helper container any time with `docker rm -f blacknode-ros2` — it is
recreated on demand.

Note: the Docker backend is a self-contained ROS graph inside the container.
It is useful for demos, learning, and agent development. The camera adapter's
`CameraROS2Subscribe` (in `blacknode-perception`) also works in this mode for image
topics that exist inside the helper container; Blacknode exposes the MJPEG
bridge on localhost using the configured stream port range, which this package
owns. To talk to host USB cameras, native robot
drivers, or robots on your LAN, use a native/WSL ROS 2 install or a rosbridge
server (DDS discovery does not cross the Docker Desktop NAT on Windows/macOS).

`CameraROS2Subscribe` follows the run mode. **Go Live** starts a continuous MJPEG
stream and emits `streaming=true`; the preview shows a `LIVE` placeholder
immediately, then live frames once the topic publishes. A plain one-shot
**Run** captures a single frame and emits `streaming=false` instead, so a
one-off run never leaves a background stream server behind. Set `action=stop`
and cook it to stop a running stream.

`ROS2Run` uses the same environment as the Blacknode server process. If your
camera driver lives in a workspace overlay, make sure that overlay is sourced
automatically by `./start.sh` or source it manually before starting Blacknode.
Then set `package`, `executable`, and optional `arguments` in the node. Cook it
with `action=start`; use the node's stop control or set `action=stop` and cook
again to stop the background process.

## Live robot control

This package is the ROS 2 transport/control layer. Use `blacknode-robot` for
generic USB discovery, permissions, driver descriptors, and driver launch. Once
a robot driver exposes a ROS-compatible joint interface, the nodes here can read
and command it through either native `rclpy` or rosbridge.

### Automatic transport selection

The regular robot nodes expose `transport=auto`. When `rclpy` is importable in
the Blacknode server environment they use the local ROS 2 graph directly.
Otherwise they use rosbridge; `ROS2Status` ensures the local rosbridge Docker
service is ready before continuing. `transport=native` and
`transport=rosbridge` remain available as advanced overrides.

`ROS2Status` and `ROS2RosbridgeServer` also expose `expose_lan`. It defaults to
`false`, which binds the Docker-published WebSocket to `127.0.0.1`. Set it to
`true` only for an intentional trusted-LAN endpoint such as the split
leader/follower template. Non-default ports receive distinct managed container
names, so a leader bridge on `9091` can run beside the local bridge on `9090`.
Rosbridge has no Blacknode pairing authentication; restrict its firewall rule
to the follower computer or trusted robot subnet.

For a sourced native ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/your_robot_ws/install/setup.bash
./start.sh
```

The selected path is reported in every node result:

```text
Blacknode -> rclpy -> /joint_states + /joint_commands -> robot driver
```

Use `Robot` for normal setup, then `ROS2Status` from this package and
`ROS2JointState` / `ROS2SetJoint` from the `blacknode-motion`
arm ROS 2 adapter. `ROS2FollowDetectionJoint`
(`blacknode-skills`) adds cube-following from a CV2 detection.

`ROS2SetJoint`'s `position` input is an **absolute target angle**, not a
delta — `position: 0` means "go to 0°," not "don't move." With `armed=false`
(the default) it still reads the live pose and computes what the clamped
target would be, so `before`/`target` show real numbers and the report reads
`PREVIEW (not armed): ...` — nothing is written to `/joint_commands` until you
set `armed=true`. Only the read (a passive subscribe) happens while disarmed;
the write (`stream_motion`) is what's actually gated.

The automatic rosbridge path starts `rosbridge_server` from the native ROS 2
installation when available. Install its package on Ubuntu with
`sudo apt-get install ros-$ROS_DISTRO-rosbridge-server`. The fallback path
builds a small ROS Jazzy image on first use and reuses the
`blacknode-rosbridge` container afterward.

| Input | Default | Meaning |
|---|---|---|
| `state_topic` | `/joint_states` | `JointState` to read the current pose from |
| `command_topic` | `/joint_commands` | `JointState` to stream position commands to (radians on the wire) |
| `config_topic` | (empty) | optional latched `std_msgs/String` JSON with `commands_allowed` + per-joint `lower`/`upper` limits |
| `units` | `radians` | `radians` (ROS standard) or `degrees` for the values you type and see |
| `detection` / `detection_url` | `{}` / empty | CV2 detection dict or live detector JSON URL with `center.x` for `ROS2FollowDetectionJoint` |
| `detection_stream` | `{}` | Latest-value stream handle from `TrackingObject`; preferred by `RobotFollow` |
| `gain` / `max_step` | `35` / `8` | convert normalized image error into a bounded actuator step |

The continuous follower is a managed runtime service. Cooking it once starts
the controller; subsequent cooks update its configuration or report status.
Frames and commands flow through the service without re-cooking the graph.
Use `action=stop` or the editor's **Streaming · Stop** control to shut it down.
Small corrections accumulate into a desired setpoint so servo friction cannot
stall tracking, while that setpoint remains bounded near measured feedback.
Stale joint subscriptions are discarded and reacquired automatically, and
stale detections or feedback suppress motion rather than using old data.
Leader-follower control applies the same recovery independently to the leader
and follower streams. A stale shared subscription is replaced for every
consumer so a restarted robot driver can resume live callbacks safely.

Capability controllers can attach a managed `std_msgs/String` control
subscription through the same native/rosbridge runtime contract. The
subscription has an explicit close path and does not turn a one-shot node cook
into a polling loop.

`roslibpy` is installed by **Install prerequisites** in the Packages tab,
`blacknode packages setup blacknode-ros2`, or `pip install roslibpy` **into the
Blacknode server environment**. Without it the nodes load and return a
structured "roslibpy not installed" result, and the Packages tab flags it.

The included **ROS 2 Motion Test** pre-fills the common topics
(`/joint_states`, `/joint_commands`, `/joint_config`) and leaves the joint name
for your robot. The same graph runs on native ROS 2 and Windows rosbridge.

**To move a real robot:**

1. Use `blacknode-robot` to discover the USB device and start the robot driver.
2. Make sure that driver publishes `/joint_states` and accepts
   `/joint_commands`.
   `ROS2Status` selects and prepares the available transport automatically.
3. In Blacknode, load **Move a Robot Joint** (ships with
   `blacknode-motion`) and press **Run** — the dashboard shows the live
   pose with `armed=false` (no motion).
4. Set the `ROS2SetJoint` node's `joint` and target `position`, then
   `armed=true`, and recook. It syncs to the current pose, ramps to the
   target, streams the command at `rate_hz`, and reports before/after angles.
5. For vision following, wire a CV2 detection into `ROS2FollowDetectionJoint`,
   set the joint name, tune `gain`, `deadband`, `invert`, and `max_step`, then
   arm it only after the preview report moves in the expected direction.

Safety, layered on top of the bridge's own torque/heartbeat gates:

- `armed=false` (default) never opens a connection or sends anything.
- A read-only bridge (no `--allow-commands`) is refused with a clear message.
- The first command always equals the current pose, so the arm never jumps.
- Targets are clamped to any limits reported on `/joint_config`.

Keep a physical power cutoff within reach and clear the workspace before arming.

## Development

Coding agents should read [`AGENTS.md`](AGENTS.md) before changing this package.
It defines transport ownership, managed-service behavior, robot-motion safety,
and verification commands.

After loading, modules are importable through Blacknode's stable alias:

```python
from blacknode.pkg.blacknode_ros2 import ros2_runtime
```

The suite in `tests/` runs automatically with `pytest` from the Blacknode repo
root. Integration tests skip cleanly without a backend; with Docker running
they exercise a real publish → echo roundtrip.

## License

Apache-2.0, same as Blacknode.
