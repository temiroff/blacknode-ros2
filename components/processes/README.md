# Processes

Local colcon workspace builds plus managed `ros2 run` and `ros2 launch`
process primitives. Connect `ROS2WorkspaceBuild.workspace_path` to the matching
input on `ROS2Run` or `ROS2Launch` to run a package from that built overlay.

Native launches are session-scoped. Blacknode stops only the child process it
started and never uses command-pattern matching against the host ROS graph.
Existing vendor bringup processes, boot services, workspaces, and configuration
remain unchanged. Runtime restart or device reboot does not resume a stopped
workflow automatically.

This component depends on `core` and owns the node registrations under
`components/processes/nodes`.
