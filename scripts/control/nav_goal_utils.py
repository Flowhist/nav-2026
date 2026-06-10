"""Small helpers shared by manual and voice navigation entrypoints."""

from geometry_msgs.msg import PoseStamped


def set_yaw_orientation(orientation, yaw_rad: float) -> None:
    """Write a planar yaw angle into a ROS quaternion-like orientation field."""
    import math

    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw_rad * 0.5)
    orientation.w = math.cos(yaw_rad * 0.5)


def make_map_goal(clock, x: float, y: float, yaw_rad: float, frame_id: str = "map") -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = clock.now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    set_yaw_orientation(msg.pose.orientation, yaw_rad)
    return msg
