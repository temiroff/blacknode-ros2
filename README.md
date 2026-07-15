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
| `ROS2ImageSnapshot` | Capture and display one raw `sensor_msgs/Image` camera frame |
| `ROS2ImageStream` | Start/stop a live MJPEG preview for a raw or compressed ROS image topic |
| `ROS2TopicPublish` | Publish one or more messages (YAML payload) to a topic |
| `ROS2DemoPublisher` | Start/stop a background publisher so you can demo without a robot |
| `ROS2Launch` | Start/stop a background `ros2 launch ...` process |
| `ROS2Run` | Start/stop a background `ros2 run <package> <executable> ...` process |
| `ROS2NodeList` | List running ROS nodes |
| `ROS2ServiceList` | List live services, optionally with types |
| `ROS2InterfaceShow` | Show a message/service definition — lets AI agents compose valid payloads |
| `ROS2PackageExecutables` | List executable commands registered by a ROS 2 package |
| `ROS2Command` | Escape hatch: run any `ros2 ...` subcommand and capture the output |
| `ROS2Status` | Auto-select native `rclpy` or rosbridge, ensuring the local rosbridge service when needed |
| `ROS2RobotDiscovery` | Detect a robot over the selected transport and output one standard robot profile |
| `ROS2JointState` | Read any robot's current pose over the selected transport |
| `ROS2SetJoint` | Set one absolute joint target; previews live before/target values while disarmed |
| `ROS2RotateJoint` | Move one joint by a relative delta over the selected transport |
| `ROS2FollowDetectionJoint` | Visual-servo one joint toward a CV2 detection center over the selected transport |
| `ROS2ContinuousFollowDetectionJoint` | Cook once to start a persistent visual-servo service with one long-lived joint-state subscription and command publisher |
| `ROS2LeaderFollower` | Stream a released leader pose into a separately calibrated follower with mapping, limits, stale-data suppression, and bounded commands |
| `ROS2MotionDashboard` | Render before/after joint values so the graph visibly shows the robot moved |

Action nodes carry an optional `trigger` input so you can sequence them in a
graph (start the publisher → then echo).

## Templates

Loadable from the editor's Templates tab:

- **ROS 2 System Check** — quick preflight with a visible backend status output
- **ROS 2 Live Roundtrip Demo** — press the top-bar **Run** button to start a
  publisher on `/blacknode_demo`, capture a real message, and render a large
  visual dashboard with the message path, pass/fail checks, and graph metrics
- **ROS 2 Camera Snapshot** — capture a raw `/camera/image_raw`
  `sensor_msgs/Image` frame and display it on the canvas
- **ROS 2 Camera Livestream** — start a live MJPEG stream from
  `/camera/image_raw` and preview it directly on the canvas
- **ROS 2 Run Camera Livestream** — start any installed camera executable with
  `ros2 run`, wait for its image topic, and preview the live stream
- **ROS 2 Launch Camera Inspector** — fill any installed ROS package and launch
  file, inspect package executables and live topics, then display one camera
  frame from `/camera/image_raw`
- **ROS 2 Motion Test** — auto-selects the available transport, reads the live
  pose, and (once you set `armed=true`) moves one joint on the real robot while
  rendering a before/after dashboard

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
It is useful for demos, learning, and agent development. `ROS2ImageSnapshot`
and `ROS2ImageStream` also work in this mode for image topics that exist inside
the helper container; Blacknode exposes the MJPEG bridge on localhost using the
configured stream port range. To talk to host USB cameras, native robot
drivers, or robots on your LAN, use a native/WSL ROS 2 install or a rosbridge
server (DDS discovery does not cross the Docker Desktop NAT on Windows/macOS).

For livestream, cook `ROS2ImageStream` with `action=start`, then switch
`action=stop` and cook it again when done. The preview shows a `LIVE`
placeholder immediately, then live frames once the topic publishes; each frame
is stamped with a small `LIVE` badge and the node also emits `streaming=true`.

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

Use `ROS2Status` first, then `ROS2RobotDiscovery` or `ROS2JointState`. Use
`ROS2SetJoint` for an absolute actuator target and
`ROS2FollowDetectionJoint` for cube-following from a CV2 detection.

`ROS2SetJoint`'s `position` input is an **absolute target angle**, not a
delta — `position: 0` means "go to 0°," not "don't move." With `armed=false`
(the default) it still reads the live pose and computes what the clamped
target would be, so `before`/`target` show real numbers and the report reads
`PREVIEW (not armed): ...` — nothing is written to `/joint_commands` until you
set `armed=true`. Only the read (a passive subscribe) happens while disarmed;
the write (`stream_motion`) is what's actually gated.

On Windows, the automatic rosbridge path starts Docker Desktop when necessary,
builds a small ROS Jazzy rosbridge image on first use, and reuses the
`blacknode-rosbridge` container afterward. Docker Desktop must be installed,
but the user does not need to choose or start rosbridge manually.

| Input | Default | Meaning |
|---|---|---|
| `state_topic` | `/joint_states` | `JointState` to read the current pose from |
| `command_topic` | `/joint_commands` | `JointState` to stream position commands to (radians on the wire) |
| `config_topic` | (empty) | optional latched `std_msgs/String` JSON with `commands_allowed` + per-joint `lower`/`upper` limits |
| `units` | `radians` | `radians` (ROS standard) or `degrees` for the values you type and see |
| `detection` / `detection_url` | `{}` / empty | CV2 detection dict or live detector JSON URL with `center.x` for `ROS2FollowDetectionJoint` |
| `detection_stream` | `{}` | Latest-value stream handle from `CV2ColorObjectStream`; preferred by `ROS2ContinuousFollowDetectionJoint` |
| `gain` / `max_step` | `35` / `8` | convert normalized image error into a bounded actuator step |

The continuous follower is a managed runtime service. Cooking it once starts
the controller; subsequent cooks update its configuration or report status.
Frames and commands flow through the service without re-cooking the graph.
Use `action=stop` or the editor's **Streaming · Stop** control to shut it down.
Small corrections accumulate into a desired setpoint so servo friction cannot
stall tracking, while that setpoint remains bounded near measured feedback.
Stale joint subscriptions are discarded and reacquired automatically, and
stale detections or feedback suppress motion rather than using old data.

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
3. In Blacknode, load **ROS 2 Live Motion Test** and press **Run** — the
   dashboard shows the live pose with `armed=false` (no motion).
4. Set the `ROS2RotateJoint` node's `armed=true` and recook. It syncs to the
   current pose, ramps the chosen joint by `delta`, streams the command at
   `rate_hz` for `hold_seconds`, and reports the before/after angles.
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

After loading, modules are importable through Blacknode's stable alias:

```python
from blacknode.pkg.blacknode_ros2 import ros2_runtime
```

The suite in `tests/` runs automatically with `pytest` from the Blacknode repo
root. Integration tests skip cleanly without a backend; with Docker running
they exercise a real publish → echo roundtrip.

## License

Apache-2.0, same as Blacknode.
