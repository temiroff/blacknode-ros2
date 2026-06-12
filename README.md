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
| `ROS2TopicPublish` | Publish one or more messages (YAML payload) to a topic |
| `ROS2DemoPublisher` | Start/stop a background publisher so you can demo without a robot |
| `ROS2NodeList` | List running ROS nodes |
| `ROS2ServiceList` | List live services, optionally with types |
| `ROS2InterfaceShow` | Show a message/service definition — lets AI agents compose valid payloads |
| `ROS2Command` | Escape hatch: run any `ros2 ...` subcommand and capture the output |

Action nodes carry an optional `trigger` input so you can sequence them in a
graph (start the publisher → then echo).

## Templates

Loadable from the editor's Templates tab:

- **ROS 2 System Check** — one node that tells you exactly how ROS runs here
- **ROS 2 Topic Explorer** — demo publisher on `/chatter` → echo 3 messages → topic list

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
