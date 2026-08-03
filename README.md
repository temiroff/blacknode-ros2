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

Use `ROS2GraphExplorer` for a read-only topology, `ROS2TopicPublisher` and `ROS2TopicSubscriber` for managed messaging, `ROS2TopicRelay` for non-motion data routing, and `ROS2Run` or `ROS2Launch` for supervised processes. Motion destinations are rejected by the generic relay; use an explicitly armed controller from `blacknode-motion`.

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
