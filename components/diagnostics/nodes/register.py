"""Register diagnostics when the diagnostics component is enabled."""
from .. import ros2, ros2_live
from .._implementation import register_implementation

for _implementation in (
    ros2.ros2_system_check,
    ros2.ros2_node_list,
    ros2.ros2_interface_show,
    ros2.ros2_visual_dashboard,
    ros2_live.ros2_status,
):
    register_implementation(_implementation)
