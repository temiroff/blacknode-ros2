"""Register process primitives when the processes component is enabled."""
from .. import ros2
from .._implementation import register_implementation

for _implementation in (
    ros2.ros2_launch,
    ros2.ros2_run,
    ros2.ros2_package_executables,
):
    register_implementation(_implementation)
