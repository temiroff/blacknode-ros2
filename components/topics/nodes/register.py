"""Register topic primitives when the topics component is enabled."""
from .. import ros2
from .._implementation import register_implementation

for _implementation in (
    ros2.ros2_topic,
    ros2.ros2_topic_list,
    ros2.ros2_topic_echo,
    ros2.ros2_topic_subscriber,
    ros2.ros2_topic_publisher,
    ros2.ros2_topic_relay,
):
    register_implementation(_implementation)
