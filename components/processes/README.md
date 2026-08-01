# Processes

Local colcon workspace builds plus managed `ros2 run` and `ros2 launch`
process primitives. Connect `ROS2WorkspaceBuild.workspace_path` to the matching
input on `ROS2Run` or `ROS2Launch` to run a package from that built overlay.

This component depends on `core` and owns the node registrations under
`components/processes/nodes`.
