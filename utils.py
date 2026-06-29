import math


def normalize_angle(angle):
    """Wrap an angle to the range -pi to pi."""
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(q):
    """Convert a ROS quaternion orientation into a yaw angle."""
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, min_value, max_value):
    """Limit a number so it stays between min_value and max_value."""
    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value


def lidar_range_to_meters(value):
    """
    Normal ROS LaserScan values are already meters.
    This robot may publish centimeters, so values larger than 20 are converted.
    """
    if value > 20.0:
        return value / 100.0

    return value
