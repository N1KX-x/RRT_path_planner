import math

from geometry_msgs.msg import Twist

from .config import (
    K_ANGULAR,
    K_LINEAR,
    MAX_ANGULAR_SPEED,
    MAX_LINEAR_SPEED,
    ROTATE_FIRST_ANGLE,
    WAYPOINT_TOLERANCE,
)
from .utils import clamp, normalize_angle


class PathFollower:
    """Simple waypoint follower that turns toward a point and drives forward."""

    def __init__(self):
        """Load controller gains and speed limits from config.py."""
        self.max_linear_speed = MAX_LINEAR_SPEED
        self.max_angular_speed = MAX_ANGULAR_SPEED
        self.waypoint_tolerance = WAYPOINT_TOLERANCE
        self.k_linear = K_LINEAR
        self.k_angular = K_ANGULAR
        self.rotate_first_angle = ROTATE_FIRST_ANGLE

    def compute_cmd_vel(self, robot_x, robot_y, robot_theta, target_x, target_y):
        """Return the velocity command needed to move toward one waypoint."""
        cmd = Twist()

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Close enough to move on to the next waypoint.
        if distance < self.waypoint_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd, True

        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - robot_theta)

        # Turn first when the target is too far off the robot's heading.
        if abs(angle_error) > self.rotate_first_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(
                self.k_angular * angle_error,
                -self.max_angular_speed,
                self.max_angular_speed
            )
            return cmd, False

        cmd.linear.x = clamp(
            self.k_linear * distance,
            0.04,
            self.max_linear_speed
        )
        cmd.angular.z = clamp(
            self.k_angular * angle_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        return cmd, False
