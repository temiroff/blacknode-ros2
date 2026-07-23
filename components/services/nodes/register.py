"""Register service primitives when the services component is enabled."""
from .. import ros2
from .._implementation import register_implementation

register_implementation(ros2.ros2_service_list)
