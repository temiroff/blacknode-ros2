# Core

Shared ROS 2 runtime contracts used by Blacknode capability adapters.

The component mounts the legacy root `nodes/` package so imports such as
`blacknode.pkg.blacknode_ros2.ros2_runtime` remain stable. It registers no
palette nodes; enabled feature components register their own node contracts.
