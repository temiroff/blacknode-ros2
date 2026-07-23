"""Register rosbridge primitives when the rosbridge component is enabled."""
from .. import ros2_live, rosbridge_service, rosbridge_topics
from .._implementation import register_implementation

for _implementation in (
    rosbridge_topics.ros2_bridge_publish,
    rosbridge_topics.ros2_bridge_echo,
    rosbridge_service.ros2_rosbridge_server,
    ros2_live.ros2_rosbridge_status,
):
    register_implementation(_implementation)
