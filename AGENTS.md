# blacknode-ros2 Agent Instructions

This is an independent extension-package repository. Check and commit its Git
state separately from the Blacknode core checkout that may contain it.

## Scope

Keep ROS 2 discovery, native `rclpy`, rosbridge, topic/service/process nodes,
ROS camera transport, joint-state transport, and ROS control nodes here. Keep
USB robot discovery and physical drivers in `blacknode-robot`; keep perception
algorithms in `blacknode-vision`.

## Development rules

- Preserve `transport=auto`: prefer a usable native ROS graph and otherwise use
  the supported rosbridge path. Keep explicit overrides available.
- Load without ROS, Docker, or rosbridge and return actionable structured errors.
- Keep motion nodes `armed=false` by default. A disarmed preview may read state
  but must not publish a command.
- The first armed command must synchronize to current pose. Clamp targets to
  reported limits and suppress motion on stale detection or joint feedback.
- Keep reconnect/retry control actions idempotent. Do not restart a healthy
  physical driver merely because a transport reconnects.
- Treat subscriptions, streams, controllers, and launched processes as managed
  services with visible status and explicit stop paths.
- Declare imports and Docker images in `blacknode-package.toml`.
- Mark templates with all required packages and keep generic node names in new
  graphs; retain compatibility names only for existing workflows.

## Verification

From the Blacknode root:

```powershell
python -m pytest packages/blacknode-ros2/tests
Get-ChildItem packages\blacknode-ros2\templates\*.json | ForEach-Object { blacknode validate $_.FullName }
```

Tests must skip cleanly when ROS or Docker is unavailable. Never claim physical
motion was tested without hardware evidence. See `docs/packages.md` and the
`blacknode-development` skill for shared package rules.

## Documentation voice

Describe Blacknode ROS nodes, transports, streams, controllers, and lifecycle
directly. Mention external names only for implemented protocols and runtime
requirements; avoid product comparisons.
