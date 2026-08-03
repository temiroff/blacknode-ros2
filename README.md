# blacknode-ros2

`blacknode-ros2` provides ROS 2 graph, topic, service, process, native `rclpy`, and rosbridge integration primitives for Blacknode.

Capability-specific nodes remain with their owning packages: cameras and LiDAR in `blacknode-perception`, robot control in `blacknode-motion`, and task behavior in `blacknode-skills`.

## Components

| Component | Default | Purpose |
|---|---:|---|
| `core` | On | Shared native ROS 2 and DDS runtime |
| `topics` | On | Topic list, echo, publish, subscribe, and relay |
| `services` | On | Service discovery |
| `diagnostics` | On | Status, graph exploration, interfaces, nodes, and dashboards |
| `rosbridge` | Off | WebSocket transport and rosbridge lifecycle |
| `processes` | Off | Workspace builds and managed `ros2 run` / `ros2 launch` processes |

## Transport behavior

`transport=auto` prefers a usable native ROS 2 graph and otherwise uses the supported rosbridge path. Explicit `native` and `rosbridge` overrides remain available. Missing ROS, Docker, or rosbridge returns a structured setup error while package discovery continues.

Use `ROS2` to configure one topic and manage it with `once`, `start`, `status`,
and `stop`. Connect `ComputeDevice.device` to `ROS2.device` to subscribe on a
paired device Runtime; leave it empty to subscribe locally. It outputs the
latest message plus portable stream and freshness status records. Advanced topic publication, relay, and discovery remain
available through the `ROS2Topic*` nodes.

Native topic workers automatically use the Python interpreter compatible with the sourced ROS 2 distribution; set `BLACKNODE_ROS2_PYTHON` only when that interpreter is outside `PATH`.

## Included workflows

- Publish and subscribe a typed message
- Relay a data topic
- Run a ROS 2 package
- Explore the live graph
- Connect to a robot over rosbridge

```powershell
blacknode packages install https://github.com/temiroff/blacknode-ros2.git
blacknode packages setup blacknode-ros2
```

## Safety and development

- Motion-capable adapters start disarmed and synchronize to current feedback before commanding.
- Stale detection, joint state, faults, stop, or shutdown suppress commands.
- Rosbridge has no Blacknode pairing authentication; expose it only on a trusted network.
- Managed subscriptions, processes, and streams have explicit stop paths.

```powershell
python -m pytest packages/blacknode-ros2/tests
Get-ChildItem packages\blacknode-ros2\templates\*.json | ForEach-Object { blacknode validate $_.FullName }
```

See [AGENTS.md](AGENTS.md) for transport ownership and managed-service rules.
