import json
import math
import os
import struct
import time
import zlib

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .occupancy_grid import OccupancyGrid
from .informed_rrt_planner import InformedRRTPlanner
from .path_follower import PathFollower
from .utils import clamp, quaternion_to_yaw, normalize_angle, lidar_range_to_meters

from .config import (
    DEFAULT_GOAL_X,
    DEFAULT_GOAL_Y,
    MAP_WIDTH_M,
    MAP_HEIGHT_M,
    GRID_RESOLUTION,
    OBSTACLE_INFLATION_RADIUS,
    GOAL_POINT_INFLATION_RADIUS,
    FRONT_OBSTACLE_STOP_DISTANCE,
    FRONT_HARD_STOP_ANGLE_DEG,
    RECOVERY_FORWARD_SPEED_TRIGGER,
    RECOVERY_TURN_TOWARD_ANGULAR_TRIGGER,
    SIDE_CLEARANCE_TRIGGER_DISTANCE,
    SIDE_CLEARANCE_ACTIVATION_ANGLE,
    SIDE_CLEARANCE_MIN_FORWARD_SPEED,
    SIDE_CLEARANCE_MAX_ANGULAR_SPEED,
    SIDE_CLEARANCE_SIDE_SECTOR_HALF_ANGLE,
    K_ANGULAR,
    PATH_REPLAN_COOLDOWN_SCANS,
    PATH_REPLAN_LOOKAHEAD_DISTANCE,
    REPLAN_STRAIGHT_SPEED,
    DYNAMIC_OBSTACLE_DETECTION_RANGE,
    DYNAMIC_OBSTACLE_CHANGED_CELL_THRESHOLD,
    DYNAMIC_OBSTACLE_DETECTION_HALF_ANGLE,
    DYNAMIC_OBSTACLE_PIXEL_RESOLUTION,
    DYNAMIC_OBSTACLE_SLOW_SPEED,
    DYNAMIC_OBSTACLE_HOLD_SCANS,
    FRONT_DETECTION_ANGLE,
    PATH_DOWNSAMPLE_STEP,
    GOAL_TOLERANCE,
    RECOVERY_REAR_CLEAR_DISTANCE,
    RECOVERY_BACKUP_SPEED,
    RECOVERY_BACKUP_TICKS,
    RECOVERY_BACKTRACK_ANGULAR_GAIN,
    RECOVERY_BACKTRACK_LOOKBACK_POINTS,
    RECOVERY_BACKTRACK_MAX_ANGULAR_SPEED,
    SAVE_TRIAL_MAP,
    TRIAL_MAP_OUTPUT_DIR,
    TRIAL_MAP_PIXEL_SCALE,
    MULTIBOT_DEFAULT,
    DEFAULT_ROBOT_ID,
    ROBOT_RADIUS,
    MULTIBOT_SHARED_ORIGIN_X,
    MULTIBOT_SHARED_ORIGIN_Y,
    MULTIBOT_SHARED_ORIGIN_YAW,
    MULTIBOT_STATE_TOPIC,
    MULTIBOT_START_TOPIC,
    MULTIBOT_CONTROL_TOPIC,
    MULTIBOT_STATE_PUBLISH_PERIOD,
    MULTIBOT_CONTROL_TIMEOUT,
    MULTIBOT_SLOW_LINEAR_SPEED,
    MULTIBOT_SAFETY_MARGIN
)


def parameter_to_bool(value):
    """Convert ROS parameter values such as true, 1, or 'yes' into bool."""
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on", "y")

    return bool(value)


class InformedRRTNavigator(Node):
    """ROS node that builds a local map, plans with Informed RRT*, and drives the robot."""

    def __init__(self):
        """Set up parameters, robot state, ROS topics, and the control timer."""
        super().__init__("informed_rrt_navigator")

        # goal_x and goal_y can be overridden with ROS parameters at launch.
        self.declare_parameter("goal_x", DEFAULT_GOAL_X)
        self.declare_parameter("goal_y", DEFAULT_GOAL_Y)
        self.declare_parameter("multibot", MULTIBOT_DEFAULT)
        self.declare_parameter("robot_id", DEFAULT_ROBOT_ID)
        self.declare_parameter("robot_radius", ROBOT_RADIUS)
        self.declare_parameter("shared_origin_x", MULTIBOT_SHARED_ORIGIN_X)
        self.declare_parameter("shared_origin_y", MULTIBOT_SHARED_ORIGIN_Y)
        self.declare_parameter("shared_origin_yaw", MULTIBOT_SHARED_ORIGIN_YAW)
        self.declare_parameter("obstacle_inflation_radius", OBSTACLE_INFLATION_RADIUS)
        self.declare_parameter(
            "goal_point_inflation_radius",
            GOAL_POINT_INFLATION_RADIUS
        )

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.multibot_enabled = parameter_to_bool(self.get_parameter("multibot").value)
        self.robot_id = str(self.get_parameter("robot_id").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.shared_origin_x = float(self.get_parameter("shared_origin_x").value)
        self.shared_origin_y = float(self.get_parameter("shared_origin_y").value)
        self.shared_origin_yaw = float(self.get_parameter("shared_origin_yaw").value)
        self.obstacle_inflation_radius = float(
            self.get_parameter("obstacle_inflation_radius").value
        )
        self.goal_point_inflation_radius = float(
            self.get_parameter("goal_point_inflation_radius").value
        )
        self.multibot_started = not self.multibot_enabled
        self.multibot_command = "RUN"
        self.multibot_reason = ""
        self.multibot_speed_limit = None
        self.last_multibot_control_time = None
        self.last_published_cmd = Twist()
        self.goal_setup_ready = not self.multibot_enabled
        self.goal_setup_id = ""
        self.other_robot_goal_points = {}
        self.active_goal_x = self.goal_x
        self.active_goal_y = self.goal_y
        self.grid = OccupancyGrid(
            width_m=MAP_WIDTH_M,
            height_m=MAP_HEIGHT_M,
            resolution=GRID_RESOLUTION,
            inflation_radius=self.obstacle_inflation_radius
        )

        self.planner = InformedRRTPlanner()
        self.follower = PathFollower()
        self.initial_pose_set = False

        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.latest_scan = None
        self.robot_trail = [(0.0, 0.0)]

        self.path = []
        self.path_index = 0
        self.need_replan = True
        self.replan_straight_mode = False
        self.path_replan_cooldown = 0
        self.previous_dynamic_obstacle_pixels = None
        self.dynamic_obstacle_hold_scans = 0
        self.adjusted_goal_cell = None
        self.goal_reached = False
        self.shutdown_requested = False
        self.recovery_mode = False
        self.recovery_counter = 0
        self.recovery_trail = []
        self.recovery_trail_index = 0
        self.normal_motion_ticks_since_start = 0

        self.backup_ticks = RECOVERY_BACKUP_TICKS
        self.backup_speed = RECOVERY_BACKUP_SPEED
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )


        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.goal_marker_pub = self.create_publisher(
            MarkerArray,
            "/goal_marker",
            10
        )

        self.multibot_state_pub = None
        self.multibot_start_sub = None
        self.multibot_control_sub = None
        self.multibot_state_timer = None

        if self.multibot_enabled:
            self.multibot_state_pub = self.create_publisher(
                String,
                MULTIBOT_STATE_TOPIC,
                10
            )
            self.multibot_start_sub = self.create_subscription(
                String,
                MULTIBOT_START_TOPIC,
                self.multibot_start_callback,
                10
            )
            self.multibot_control_sub = self.create_subscription(
                String,
                MULTIBOT_CONTROL_TOPIC,
                self.multibot_control_callback,
                10
            )
            self.multibot_state_timer = self.create_timer(
                MULTIBOT_STATE_PUBLISH_PERIOD,
                self.publish_multibot_state
            )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Informed RRT* Navigator started.")
        self.get_logger().info(
            f"Target point coordinate: x={self.goal_x:.2f}, y={self.goal_y:.2f} meters"
        )

        if self.multibot_enabled:
            self.get_logger().info(
                f"Multibot enabled for robot_id={self.robot_id}. "
                "Waiting for central START command."
            )
            self.get_logger().info(
                "Shared traffic origin: "
                f"x={self.shared_origin_x:.2f}, y={self.shared_origin_y:.2f}, "
                f"yaw={self.shared_origin_yaw:.2f} rad"
            )


    def publish_goal_marker(self):
        """Publish a red goal marker in RViz at the current target coordinate."""
        # Delete the old marker first so RViz only shows the current target.
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = "odom"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "goal"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        pin = Marker()
        pin.header.frame_id = "odom"
        pin.header.stamp = self.get_clock().now().to_msg()
        pin.ns = "goal"
        pin.id = 1
        pin.type = Marker.CYLINDER
        pin.action = Marker.ADD
        pin.pose.position.x = self.goal_x
        pin.pose.position.y = self.goal_y
        pin.pose.position.z = 0.25
        pin.pose.orientation.w = 1.0
        pin.scale.x = 0.03
        pin.scale.y = 0.03
        pin.scale.z = 0.50
        pin.color.r = 1.0
        pin.color.g = 0.0
        pin.color.b = 0.0
        pin.color.a = 1.0
        marker_array.markers.append(pin)

        disk = Marker()
        disk.header.frame_id = "odom"
        disk.header.stamp = self.get_clock().now().to_msg()
        disk.ns = "goal"
        disk.id = 2
        disk.type = Marker.CYLINDER
        disk.action = Marker.ADD
        disk.pose.position.x = self.goal_x
        disk.pose.position.y = self.goal_y
        disk.pose.position.z = 0.01
        disk.pose.orientation.w = 1.0
        disk.scale.x = 0.12
        disk.scale.y = 0.12
        disk.scale.z = 0.01
        disk.color.r = 1.0
        disk.color.g = 0.0
        disk.color.b = 0.0
        disk.color.a = 0.7
        marker_array.markers.append(disk)

        text = Marker()
        text.header.frame_id = "odom"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "goal"
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.goal_x
        text.pose.position.y = self.goal_y
        text.pose.position.z = 0.75
        text.pose.orientation.w = 1.0
        text.scale.z = 0.20
        text.text = "GOAL"
        text.color.r = 1.0
        text.color.g = 0.0
        text.color.b = 0.0
        text.color.a = 1.0
        marker_array.markers.append(text)

        self.goal_marker_pub.publish(marker_array)

    def multibot_start_callback(self, msg):
        """Receive start/stop commands from the central multibot coordinator."""
        command = self.parse_multibot_command(msg.data).get("command", msg.data)
        command = str(command).strip().upper()

        if command in ("START", "RUN", "RESUME", "Y"):
            if not self.multibot_started:
                self.get_logger().info("Central START received. Navigation armed.")

            self.multibot_started = True
            self.multibot_command = "RUN"
            self.multibot_reason = "central_start"
            return

        if command in ("STOP", "PAUSE", "HOLD"):
            self.multibot_started = False
            self.multibot_command = "HOLD"
            self.multibot_reason = "central_pause"
            self.stop_robot()

    def multibot_control_callback(self, msg):
        """Receive RUN/SLOW/HOLD/EMERGENCY_STOP commands from the coordinator."""
        data = self.parse_multibot_command(msg.data)
        target = str(data.get("robot_id", data.get("target", "*")))

        if target not in ("*", self.robot_id):
            return

        command = str(data.get("command", "RUN")).strip().upper()

        if command == "SETUP_GOALS":
            self.apply_multibot_goal_setup(data)
            return

        if command not in ("RUN", "SLOW", "HOLD", "EMERGENCY_STOP"):
            return

        self.multibot_command = command
        self.multibot_reason = str(data.get("reason", ""))
        self.last_multibot_control_time = time.monotonic()

        if "speed_limit" in data:
            self.multibot_speed_limit = float(data["speed_limit"])
        else:
            self.multibot_speed_limit = None

    def parse_multibot_command(self, text):
        """Parse JSON coordination messages, with plain text as a fallback."""
        try:
            data = json.loads(text)
        except (TypeError, ValueError):
            return {"command": text}

        if isinstance(data, dict):
            return data

        return {"command": text}

    def local_point_to_shared(self, x, y):
        """Transform a local planner point into the shared traffic frame."""
        cos_t = math.cos(self.shared_origin_yaw)
        sin_t = math.sin(self.shared_origin_yaw)
        shared_x = self.shared_origin_x + x * cos_t - y * sin_t
        shared_y = self.shared_origin_y + x * sin_t + y * cos_t

        return shared_x, shared_y

    def local_pose_to_shared(self, x, y, theta):
        """Transform a local planner pose into the shared traffic frame."""
        shared_x, shared_y = self.local_point_to_shared(x, y)
        shared_theta = normalize_angle(theta + self.shared_origin_yaw)

        return shared_x, shared_y, shared_theta

    def shared_point_to_local(self, x, y):
        """Transform a shared multibot point into this robot's local map frame."""
        dx = x - self.shared_origin_x
        dy = y - self.shared_origin_y
        cos_t = math.cos(self.shared_origin_yaw)
        sin_t = math.sin(self.shared_origin_yaw)

        local_x = dx * cos_t + dy * sin_t
        local_y = -dx * sin_t + dy * cos_t
        return local_x, local_y

    def apply_multibot_goal_setup(self, data):
        """Reserve every other robot's final goal in this robot's local map."""
        setup_id = str(data.get("setup_id", ""))
        raw_goals = data.get("goals", [])

        if not setup_id or not isinstance(raw_goals, list):
            self.goal_setup_ready = False
            return

        try:
            safety_margin = max(
                0.0,
                float(data.get("safety_margin", MULTIBOT_SAFETY_MARGIN))
            )
        except (TypeError, ValueError):
            self.goal_setup_ready = False
            return

        if self.goal_setup_ready and setup_id == self.goal_setup_id:
            self.mark_other_robot_goals()
            return

        transformed_goals = {}

        for goal in raw_goals:
            if not isinstance(goal, dict):
                continue

            other_robot_id = str(goal.get("robot_id", "")).strip()
            if not other_robot_id or other_robot_id == self.robot_id:
                continue

            try:
                shared_x = float(goal["x"])
                shared_y = float(goal["y"])
                other_robot_radius = max(
                    0.0,
                    float(goal.get("robot_radius", ROBOT_RADIUS))
                )
            except (KeyError, TypeError, ValueError):
                self.goal_setup_ready = False
                return

            local_x, local_y = self.shared_point_to_local(shared_x, shared_y)
            cell = self.grid.world_to_grid(local_x, local_y)

            if cell is None:
                self.get_logger().error(
                    f"Cannot mark {other_robot_id} goal: local point "
                    f"({local_x:.2f}, {local_y:.2f}) is outside this robot's map."
                )
                self.goal_setup_ready = False
                return

            effective_radius = max(
                self.goal_point_inflation_radius,
                self.robot_radius + other_robot_radius + safety_margin
            )
            transformed_goals[other_robot_id] = (
                local_x,
                local_y,
                effective_radius
            )

        self.other_robot_goal_points = transformed_goals
        self.mark_other_robot_goals()
        self.goal_setup_id = setup_id
        self.goal_setup_ready = True
        self.need_replan = True
        self.replan_straight_mode = False

        marked_ids = ", ".join(sorted(transformed_goals)) or "none"
        self.get_logger().info(
            f"Multibot goal setup ready. Marked goal obstacles for: {marked_ids}."
        )

    def mark_other_robot_goals(self):
        """Reapply reserved final goals so LiDAR ray clearing cannot erase them."""
        for (
            local_x,
            local_y,
            effective_radius
        ) in self.other_robot_goal_points.values():
            cell = self.grid.world_to_grid(local_x, local_y)
            if cell is not None:
                self.grid.inflate_obstacle(
                    cell,
                    inflation_radius=effective_radius
                )

    def publish_multibot_state(self):
        """Publish this robot's current pose, path, and coordination status."""
        if not self.multibot_enabled or self.multibot_state_pub is None:
            return

        shared_x, shared_y, shared_theta = self.local_pose_to_shared(
            self.robot_x,
            self.robot_y,
            self.robot_theta
        )
        shared_goal_x, shared_goal_y = self.local_point_to_shared(
            self.goal_x,
            self.goal_y
        )
        shared_active_goal_x, shared_active_goal_y = self.local_point_to_shared(
            self.active_goal_x,
            self.active_goal_y
        )
        remaining_path = []

        for point in self.path[self.path_index:self.path_index + 20]:
            shared_path_x, shared_path_y = self.local_point_to_shared(
                point[0],
                point[1]
            )
            remaining_path.append({"x": shared_path_x, "y": shared_path_y})

        state = {
            "robot_id": self.robot_id,
            "pose_ready": self.initial_pose_set,
            "x": shared_x,
            "y": shared_y,
            "theta": shared_theta,
            "local_x": self.robot_x,
            "local_y": self.robot_y,
            "local_theta": self.robot_theta,
            "linear_x": self.last_published_cmd.linear.x,
            "angular_z": self.last_published_cmd.angular.z,
            "robot_radius": self.robot_radius,
            "goal_x": shared_goal_x,
            "goal_y": shared_goal_y,
            "active_goal_x": shared_active_goal_x,
            "active_goal_y": shared_active_goal_y,
            "shared_origin_x": self.shared_origin_x,
            "shared_origin_y": self.shared_origin_y,
            "shared_origin_yaw": self.shared_origin_yaw,
            "started": self.multibot_started,
            "central_command": self.multibot_command,
            "central_reason": self.multibot_reason,
            "goal_setup_ready": self.goal_setup_ready,
            "goal_setup_id": self.goal_setup_id,
            "status": self.get_multibot_status(),
            "path_index": self.path_index,
            "path_length": len(self.path),
            "path": remaining_path,
        }

        msg = String()
        msg.data = json.dumps(state, separators=(",", ":"))
        self.multibot_state_pub.publish(msg)

    def get_multibot_status(self):
        """Return a compact status string for the central coordinator."""
        if not self.initial_pose_set:
            return "WAITING_FOR_ODOM"

        if self.goal_reached:
            return "GOAL_REACHED"

        if self.recovery_mode:
            return "RECOVERY"

        if self.multibot_enabled and not self.multibot_started:
            return "WAITING_FOR_START"

        if self.multibot_enabled and self.multibot_control_is_stale():
            return "CONTROL_TIMEOUT"

        if self.multibot_enabled and self.multibot_command != "RUN":
            return self.multibot_command

        if self.need_replan:
            return "PLANNING"

        return "RUNNING"

    def multibot_control_is_stale(self):
        """Return True if central control messages have timed out."""
        if self.last_multibot_control_time is None:
            return True

        return (
            time.monotonic() - self.last_multibot_control_time
        ) > MULTIBOT_CONTROL_TIMEOUT

    def publish_cmd(self, cmd):
        """Publish a Twist after applying central multibot safety limits."""
        published_cmd = self.apply_multibot_command(cmd)

        if self.dynamic_obstacle_hold_scans > 0:
            published_cmd.linear.x = clamp(
                published_cmd.linear.x,
                -DYNAMIC_OBSTACLE_SLOW_SPEED,
                DYNAMIC_OBSTACLE_SLOW_SPEED
            )

        self.cmd_pub.publish(published_cmd)
        self.last_published_cmd = published_cmd
        return published_cmd

    def apply_multibot_command(self, cmd):
        """Return a command limited by the central coordinator."""
        limited_cmd = Twist()
        limited_cmd.linear.x = cmd.linear.x
        limited_cmd.linear.y = cmd.linear.y
        limited_cmd.linear.z = cmd.linear.z
        limited_cmd.angular.x = cmd.angular.x
        limited_cmd.angular.y = cmd.angular.y
        limited_cmd.angular.z = cmd.angular.z

        if not self.multibot_enabled:
            return limited_cmd

        if not self.multibot_started:
            return Twist()

        if self.multibot_control_is_stale():
            return Twist()

        if self.multibot_command in ("HOLD", "EMERGENCY_STOP"):
            return Twist()

        if self.multibot_command == "SLOW":
            speed_limit = self.multibot_speed_limit

            if speed_limit is None:
                speed_limit = MULTIBOT_SLOW_LINEAR_SPEED

            limited_cmd.linear.x = clamp(
                limited_cmd.linear.x,
                -abs(speed_limit),
                abs(speed_limit)
            )

        return limited_cmd

    def odom_callback(self, msg):
        """Convert odometry into the local start frame used by the planner."""
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if not self.initial_pose_set:
            # First odom message becomes the local origin for this run.
            self.initial_x = odom_x
            self.initial_y = odom_y
            self.initial_theta = yaw
            self.initial_pose_set = True
            self.get_logger().info("Initial robot pose saved as origin (0, 0).")
            return

        dx = odom_x - self.initial_x
        dy = odom_y - self.initial_y

        cos_t = math.cos(-self.initial_theta)
        sin_t = math.sin(-self.initial_theta)

        # Rotate odom into the robot's starting frame: +x forward, +y left.
        self.robot_x = dx * cos_t - dy * sin_t
        self.robot_y = dx * sin_t + dy * cos_t
        self.robot_theta = normalize_angle(yaw - self.initial_theta)
        self.record_robot_trail_point()

    def scan_callback(self, msg):
        """Update the occupancy grid with LiDAR data and start recovery if blocked."""
        self.latest_scan = msg

        if not self.initial_pose_set:
            return

        if self.path_replan_cooldown > 0:
            self.path_replan_cooldown -= 1

        if self.dynamic_obstacle_hold_scans > 0:
            self.dynamic_obstacle_hold_scans -= 1

        current_dynamic_obstacle_pixels = self.grid.lidar_hit_pixels_in_sector(
            scan_msg=msg,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_theta=self.robot_theta,
            max_distance=DYNAMIC_OBSTACLE_DETECTION_RANGE,
            half_angle=DYNAMIC_OBSTACLE_DETECTION_HALF_ANGLE,
            pixel_resolution=DYNAMIC_OBSTACLE_PIXEL_RESOLUTION
        )

        self.grid.update_from_lidar(
            scan_msg=msg,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_theta=self.robot_theta
        )
        self.mark_other_robot_goals()

        if self.previous_dynamic_obstacle_pixels is None:
            # Building the map for the first time is not obstacle motion.
            changed_cell_count = 0
        else:
            changed_cell_count = len(
                self.previous_dynamic_obstacle_pixels.symmetric_difference(
                    current_dynamic_obstacle_pixels
                )
            )

        self.previous_dynamic_obstacle_pixels = current_dynamic_obstacle_pixels

        if changed_cell_count >= DYNAMIC_OBSTACLE_CHANGED_CELL_THRESHOLD:
            self.dynamic_obstacle_hold_scans = DYNAMIC_OBSTACLE_HOLD_SCANS

            if (
                not self.recovery_mode
                and not self.need_replan
                and self.path_replan_cooldown == 0
            ):
                self.get_logger().warn(
                    "Dynamic obstacle detected in front hemisphere: "
                    f"{changed_cell_count} map cells changed within "
                    f"{DYNAMIC_OBSTACLE_DETECTION_RANGE:.2f} m. "
                    "Slowing down and replanning."
                )
                self.need_replan = True
                self.replan_straight_mode = True
                self.path_replan_cooldown = PATH_REPLAN_COOLDOWN_SCANS

        if (
            not self.recovery_mode
            and not self.need_replan
            and self.path_blocked_ahead()
        ):
            self.get_logger().warn(
                "Current path is blocked by obstacle. Driving straight slowly while replanning."
            )
            self.need_replan = True
            self.replan_straight_mode = True
            self.path_replan_cooldown = PATH_REPLAN_COOLDOWN_SCANS

    def control_loop(self):
        """Main navigation loop: stop, recover, plan, or follow the next waypoint."""

        if not self.initial_pose_set:
            return

        self.publish_goal_marker()

        if self.goal_reached:
            self.stop_robot()
            return

        if self.multibot_enabled and not self.multibot_started:
            self.stop_robot()
            return

        if self.recovery_mode:
            self.run_recovery()
            return

        distance_to_goal = self.distance_to_active_goal()

        if distance_to_goal < GOAL_TOLERANCE:
            self.finish_goal()
            return

        if self.need_replan:
            # Wait for at least one LiDAR update before planning so the first
            # RRT path is based on the current occupancy grid, not an empty map.
            if self.latest_scan is None:
                self.stop_robot()
                return

            if self.replan_straight_mode:
                if not self.publish_replan_straight_cmd():
                    self.run_recovery()
                    return

            success = self.plan_path()

            if not success:
                if self.replan_straight_mode:
                    self.get_logger().warn(
                        "No new path found yet. Continuing straight slowly."
                    )
                else:
                    self.get_logger().warn("No path found. Robot stopped.")
                    self.stop_robot()
                return

            self.need_replan = False
            self.replan_straight_mode = False

        if not self.path:
            self.stop_robot()
            return

        if self.path_index >= len(self.path):
            final_tolerance = max(GOAL_TOLERANCE, self.follower.waypoint_tolerance)

            if distance_to_goal < final_tolerance:
                self.finish_goal()
                return

            # The last waypoint was reached, but not close enough to the current goal.
            self.get_logger().warn(
                f"Path ended {distance_to_goal:.2f} m from goal. Replanning."
            )
            self.path = []
            self.path_index = 0
            self.need_replan = True
            self.replan_straight_mode = False
            self.stop_robot()
            return

        target = self.path[self.path_index]

        cmd, reached_waypoint = self.follower.compute_cmd_vel(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_theta=self.robot_theta,
            target_x=target[0],
            target_y=target[1]
        )

        if reached_waypoint:
            self.path_index += 1
            return

        if self.latest_scan is not None and self.should_do_smooth_side_turn(
            self.latest_scan,
            target[0],
            target[1]
        ):
            cmd = self.compute_smooth_side_turn_cmd(
                self.latest_scan,
                target[0],
                target[1],
                cmd
            )

        if self.latest_scan is not None and self.command_needs_recovery(
            self.latest_scan,
            cmd
        ):
            self.enter_recovery()
            return

        published_cmd = self.publish_cmd(cmd)
        self.record_normal_motion_tick(published_cmd)

    def enter_recovery(self):
        """Start recovery mode after the front LiDAR sector becomes unsafe."""
        if self.recovery_mode:
            return

        self.recovery_mode = True
        self.recovery_counter = 0
        self.backup_ticks = min(
            RECOVERY_BACKUP_TICKS,
            self.normal_motion_ticks_since_start
        )
        self.need_replan = True
        self.replan_straight_mode = False
        self.recovery_trail = list(self.robot_trail)
        self.recovery_trail_index = max(
            0,
            len(self.recovery_trail) - 1 - RECOVERY_BACKTRACK_LOOKBACK_POINTS
        )

        # Mark the blocked area before replanning so the next path avoids it.
        self.block_front_danger_zone()

        self.get_logger().warn(
            f"Entering recovery: following previous trail backward for {self.backup_ticks} ticks."
        )

    def run_recovery(self):
        """Follow the previous robot trail backward, then allow the planner to replan."""
        cmd = Twist()

        if self.recovery_counter >= self.backup_ticks:
            self.get_logger().info("Recovery backup finished. Replanning.")
            self.finish_recovery()
            return

        if self.latest_scan is None or not self.rear_is_clear(self.latest_scan):
            self.get_logger().warn("Recovery: rear blocked. Stopping and replanning.")
            self.finish_recovery()
            return

        target = self.get_recovery_backtrack_target()
        cmd.linear.x = self.backup_speed
        cmd.angular.z = self.compute_backtrack_angular_velocity(target)
        published_cmd = self.publish_cmd(cmd)

        moving = (
            abs(published_cmd.linear.x) > 1e-6
            or abs(published_cmd.angular.z) > 1e-6
        )

        if moving:
            self.recovery_counter += 1

        if self.recovery_counter >= self.backup_ticks:
            self.get_logger().info("Recovery backup finished. Replanning.")
            self.finish_recovery()
            return

    def get_recovery_backtrack_target(self):
        """Return the next older trail point to backtrack toward."""
        if len(self.recovery_trail) == 0:
            return self.robot_x, self.robot_y

        while self.recovery_trail_index > 0:
            target_x, target_y = self.recovery_trail[self.recovery_trail_index]
            distance = math.sqrt(
                (target_x - self.robot_x) ** 2 +
                (target_y - self.robot_y) ** 2
            )

            if distance > self.follower.waypoint_tolerance:
                return target_x, target_y

            self.recovery_trail_index -= 1

        return self.recovery_trail[0]

    def compute_backtrack_angular_velocity(self, target):
        """Steer the robot's rear toward an older trail point while backing up."""
        target_x, target_y = target
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < 1e-6:
            return 0.0

        target_angle = math.atan2(dy, dx)
        rear_heading = normalize_angle(self.robot_theta + math.pi)
        angle_error = normalize_angle(target_angle - rear_heading)

        return clamp(
            RECOVERY_BACKTRACK_ANGULAR_GAIN * angle_error,
            -RECOVERY_BACKTRACK_MAX_ANGULAR_SPEED,
            RECOVERY_BACKTRACK_MAX_ANGULAR_SPEED
        )

    def finish_recovery(self):
        """Stop recovery and request a fresh path from the new robot position."""
        self.stop_robot()
        self.recovery_mode = False
        self.recovery_counter = 0
        self.recovery_trail = []
        self.recovery_trail_index = 0
        self.path = []
        self.path_index = 0
        self.need_replan = True

    def block_front_danger_zone(self):
        """Mark cells in front of the robot as blocked before planning again."""
        block_distance = 0.90
        half_angle = math.radians(35.0)

        d = 0.15

        while d <= block_distance:
            angle_offset = -half_angle

            while angle_offset <= half_angle:
                world_angle = self.robot_theta + angle_offset

                x = self.robot_x + d * math.cos(world_angle)
                y = self.robot_y + d * math.sin(world_angle)

                cell = self.grid.world_to_grid(x, y)

                if cell is not None:
                    self.grid.inflate_obstacle(cell)

                angle_offset += math.radians(5.0)

            d += self.grid.resolution

    def plan_path(self):
        """Plan a new path from the robot's current cell to the goal cell."""
        start_x = self.robot_x
        start_y = self.robot_y
        start_cell = self.grid.world_to_grid(self.robot_x, self.robot_y)
        requested_goal_cell = self.grid.world_to_grid(self.goal_x, self.goal_y)

        if start_cell is None:
            self.get_logger().warn("Start is outside map.")
            return False

        if requested_goal_cell is None:
            self.get_logger().warn("Goal is outside map.")
            return False

        # Keep the robot's current cell clear so planning can start after recovery.
        self.clear_cell_radius(start_cell, radius_cells=3)

        goal_cell = requested_goal_cell
        self.active_goal_x = self.goal_x
        self.active_goal_y = self.goal_y

        if not self.grid.is_free_cell(goal_cell):
            # If the exact goal is occupied, keep one stable nearby free goal.
            if (
                self.adjusted_goal_cell is not None
                and self.grid.is_free_cell(self.adjusted_goal_cell)
            ):
                new_goal_cell = self.adjusted_goal_cell
            else:
                new_goal_cell = self.find_nearest_free_cell(
                    goal_cell,
                    max_radius_cells=15,
                    prefer_x=self.robot_x,
                    prefer_y=self.robot_y
                )

            if new_goal_cell is None:
                self.get_logger().warn("Goal cell is occupied and no nearby free cell was found.")
                return False

            old_goal_world = self.grid.grid_to_world(goal_cell)
            new_goal_world = self.grid.grid_to_world(new_goal_cell)

            self.get_logger().warn(
                f"Goal cell is occupied near ({old_goal_world[0]:.2f}, {old_goal_world[1]:.2f}). "
                f"Using nearby free goal ({new_goal_world[0]:.2f}, {new_goal_world[1]:.2f}) instead."
            )

            goal_cell = new_goal_cell
            self.adjusted_goal_cell = new_goal_cell
            self.active_goal_x = new_goal_world[0]
            self.active_goal_y = new_goal_world[1]
        else:
            self.adjusted_goal_cell = None

        cell_path = self.planner.plan(
            grid=self.grid.grid,
            start=start_cell,
            goal=goal_cell
        )

        if cell_path is None or len(cell_path) == 0:
            return False

        world_path = []

        for cell in cell_path:
            x, y = self.grid.grid_to_world(cell)
            world_path.append((x, y))

        step = max(1, PATH_DOWNSAMPLE_STEP)

        planned_path = self.downsample_path(
            world_path,
            step=step
        )

        if len(planned_path) == 0:
            return False

        planned_path[0] = (start_x, start_y)
        self.path = planned_path
        self.path_index = 0

        self.get_logger().info(
            f"New path planned with {len(self.path)} waypoints."
        )

        return True

    def publish_replan_straight_cmd(self):
        """Creep straight while a blocked original path is being replanned."""
        cmd = Twist()
        cmd.linear.x = REPLAN_STRAIGHT_SPEED
        cmd.angular.z = 0.0

        if self.latest_scan is not None and self.command_needs_recovery(
            self.latest_scan,
            cmd
        ):
            self.enter_recovery()
            return False

        published_cmd = self.publish_cmd(cmd)
        self.record_normal_motion_tick(published_cmd)
        return True

    def record_normal_motion_tick(self, cmd):
        """Count normal movement since startup for early recovery duration."""
        if abs(cmd.linear.x) > 1e-6 or abs(cmd.angular.z) > 1e-6:
            self.normal_motion_ticks_since_start += 1

    def clear_cell_radius(self, center_cell, radius_cells=3):
        """Clear a small circle in the grid so the robot can plan from its own cell."""
        center_row, center_col = center_cell

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                row = center_row + dr
                col = center_col + dc

                if row < 0 or row >= self.grid.rows:
                    continue

                if col < 0 or col >= self.grid.cols:
                    continue

                if math.sqrt(dr ** 2 + dc ** 2) <= radius_cells:
                    self.grid.grid[row][col] = 0

    def find_nearest_free_cell(self, blocked_cell, max_radius_cells=15, prefer_x=None, prefer_y=None):
        """Search outward from a blocked goal and return a stable nearby free cell."""
        center_row, center_col = blocked_cell

        if self.grid.is_free_cell(blocked_cell):
            return blocked_cell

        best_cell = None
        best_distance = None

        for radius in range(1, max_radius_cells + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    row = center_row + dr
                    col = center_col + dc

                    if row < 0 or row >= self.grid.rows:
                        continue

                    if col < 0 or col >= self.grid.cols:
                        continue

                    cell = (row, col)

                    if not self.grid.is_free_cell(cell):
                        continue

                    if prefer_x is None or prefer_y is None:
                        distance = math.sqrt(dr ** 2 + dc ** 2)
                    else:
                        cell_x, cell_y = self.grid.grid_to_world(cell)
                        distance = math.sqrt(
                            (cell_x - prefer_x) ** 2 +
                            (cell_y - prefer_y) ** 2
                        )

                    if best_distance is None or distance < best_distance:
                        best_distance = distance
                        best_cell = cell

            if best_cell is not None:
                return best_cell

        return None

    def downsample_path(self, path, step=1):
        """Keep fewer waypoints so the follower gets a simpler path to track."""
        if len(path) <= 2:
            return path

        step = max(1, step)

        new_path = path[::step]

        if new_path[-1] != path[-1]:
            new_path.append(path[-1])

        return new_path

    def distance_to_active_goal(self):
        """Return distance from the robot to the goal used by the current plan."""
        return math.sqrt(
            (self.active_goal_x - self.robot_x) ** 2 +
            (self.active_goal_y - self.robot_y) ** 2
        )

    def path_blocked_ahead(self):
        """Return True when a known obstacle occupies a nearby planned path cell."""
        if len(self.path) == 0:
            return False

        if self.path_index >= len(self.path):
            return False

        if self.path_replan_cooldown > 0:
            return False

        final_tolerance = max(GOAL_TOLERANCE, self.follower.waypoint_tolerance)

        if self.distance_to_active_goal() < final_tolerance:
            return False

        previous_point = (self.robot_x, self.robot_y)
        remaining_distance = PATH_REPLAN_LOOKAHEAD_DISTANCE

        for index in range(self.path_index, len(self.path)):
            next_point = self.path[index]
            distance_to_goal_point = math.sqrt(
                (next_point[0] - self.active_goal_x) ** 2 +
                (next_point[1] - self.active_goal_y) ** 2
            )

            if distance_to_goal_point < final_tolerance:
                return False

            distance_to_next = math.sqrt(
                (next_point[0] - previous_point[0]) ** 2 +
                (next_point[1] - previous_point[1]) ** 2
            )
            remaining_distance -= distance_to_next

            cell = self.grid.world_to_grid(next_point[0], next_point[1])

            if cell is not None and not self.grid.is_free_cell(cell):
                return True

            if remaining_distance <= 0.0:
                return False

            previous_point = next_point

        return False

    def get_sector_min(self, scan_msg, start_deg, end_deg):
        """Return the nearest valid LiDAR reading inside an angle range."""
        values = []

        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)

        angle = scan_msg.angle_min

        for raw_r in scan_msg.ranges:
            normalized_angle = math.atan2(math.sin(angle), math.cos(angle))

            in_sector = False

            if start_rad <= end_rad:
                if start_rad <= normalized_angle <= end_rad:
                    in_sector = True
            else:
                if normalized_angle >= start_rad or normalized_angle <= end_rad:
                    in_sector = True

            if in_sector:
                if not math.isinf(raw_r) and not math.isnan(raw_r):
                    r = lidar_range_to_meters(raw_r)

                    if r > 0.0:
                        values.append(r)

            angle += scan_msg.angle_increment

        if len(values) == 0:
            return 999.0

        return min(values)

    def get_side_clearance_info(self, scan_msg):
        """Return the nearest left and right distances, plus the closer side name."""
        left_min = self.get_sector_min(
            scan_msg,
            90.0 - SIDE_CLEARANCE_SIDE_SECTOR_HALF_ANGLE,
            90.0 + SIDE_CLEARANCE_SIDE_SECTOR_HALF_ANGLE
        )
        right_min = self.get_sector_min(
            scan_msg,
            -90.0 - SIDE_CLEARANCE_SIDE_SECTOR_HALF_ANGLE,
            -90.0 + SIDE_CLEARANCE_SIDE_SECTOR_HALF_ANGLE
        )

        if left_min <= right_min:
            return left_min, right_min, "left"

        return left_min, right_min, "right"

    def should_do_smooth_side_turn(self, scan_msg, target_x, target_y):
        """Return True when a side obstacle is close and the robot is actively turning."""
        left_min, right_min, close_side = self.get_side_clearance_info(scan_msg)
        side_min = left_min if close_side == "left" else right_min

        if side_min > SIDE_CLEARANCE_TRIGGER_DISTANCE:
            return False

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - self.robot_theta)

        return abs(angle_error) > SIDE_CLEARANCE_ACTIVATION_ANGLE

    def compute_smooth_side_turn_cmd(self, scan_msg, target_x, target_y, base_cmd):
        """Return a low-speed forward arc command for safe side-wall turning."""
        cmd = Twist()
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - self.robot_theta)

        cmd.linear.x = SIDE_CLEARANCE_MIN_FORWARD_SPEED
        cmd.angular.z = clamp(
            K_ANGULAR * angle_error,
            -SIDE_CLEARANCE_MAX_ANGULAR_SPEED,
            SIDE_CLEARANCE_MAX_ANGULAR_SPEED
        )

        return cmd

    def get_sector_min_with_angle(self, scan_msg, start_deg, end_deg):
        """Return nearest valid LiDAR reading and its angle inside an angle range."""
        best_range = 999.0
        best_angle = 0.0

        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)

        angle = scan_msg.angle_min

        for raw_r in scan_msg.ranges:
            normalized_angle = math.atan2(math.sin(angle), math.cos(angle))

            in_sector = False

            if start_rad <= end_rad:
                if start_rad <= normalized_angle <= end_rad:
                    in_sector = True
            else:
                if normalized_angle >= start_rad or normalized_angle <= end_rad:
                    in_sector = True

            if in_sector:
                if not math.isinf(raw_r) and not math.isnan(raw_r):
                    r = lidar_range_to_meters(raw_r)

                    if 0.0 < r < best_range:
                        best_range = r
                        best_angle = normalized_angle

            angle += scan_msg.angle_increment

        return best_range, best_angle

    def command_needs_recovery(self, scan_msg, cmd):
        """Return True when the current command would move toward a close obstacle."""
        hard_front_min = self.get_sector_min(
            scan_msg,
            -FRONT_HARD_STOP_ANGLE_DEG,
            FRONT_HARD_STOP_ANGLE_DEG
        )

        if hard_front_min < FRONT_OBSTACLE_STOP_DISTANCE:
            self.get_logger().warn(
                f"Obstacle too close straight ahead: {hard_front_min:.2f} m"
            )
            return True

        wide_front_min, wide_front_angle = self.get_sector_min_with_angle(
            scan_msg,
            -math.degrees(FRONT_DETECTION_ANGLE),
            math.degrees(FRONT_DETECTION_ANGLE)
        )

        if wide_front_min >= FRONT_OBSTACLE_STOP_DISTANCE:
            return False

        if cmd.linear.x > RECOVERY_FORWARD_SPEED_TRIGGER:
            self.get_logger().warn(
                f"Obstacle too close while moving forward: {wide_front_min:.2f} m"
            )
            return True

        turning_toward_obstacle = (
            abs(cmd.angular.z) > RECOVERY_TURN_TOWARD_ANGULAR_TRIGGER
            and cmd.angular.z * wide_front_angle > 0.0
        )

        if turning_toward_obstacle:
            self.get_logger().warn(
                f"Obstacle too close while turning toward it: {wide_front_min:.2f} m"
            )
            return True

        return False

    def obstacle_too_close(self, scan_msg):
        """Check whether the front sector has an obstacle inside the stop distance."""
        min_front = self.get_sector_min(
            scan_msg,
            -math.degrees(FRONT_DETECTION_ANGLE),
            math.degrees(FRONT_DETECTION_ANGLE)
        )

        if min_front < FRONT_OBSTACLE_STOP_DISTANCE:
            self.get_logger().warn(
                f"Obstacle too close: {min_front:.2f} m"
            )
            return True

        return False

    def rear_is_clear(self, scan_msg):
        """Check whether the robot has enough space behind it to back up."""
        min_rear = self.get_sector_min(scan_msg, 135.0, -135.0)
        return min_rear > RECOVERY_REAR_CLEAR_DISTANCE

    def stop_robot(self):
        """Publish a zero Twist command."""
        if not rclpy.ok():
            return

        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.last_published_cmd = cmd

    def record_robot_trail_point(self):
        """Store the robot position occasionally for the final trial map."""
        if len(self.robot_trail) == 0:
            self.robot_trail.append((self.robot_x, self.robot_y))
            return

        last_x, last_y = self.robot_trail[-1]
        distance = math.sqrt(
            (self.robot_x - last_x) ** 2 +
            (self.robot_y - last_y) ** 2
        )

        if distance >= self.grid.resolution:
            self.robot_trail.append((self.robot_x, self.robot_y))

    def save_trial_map(self):
        """Save the final occupancy grid, path, robot trail, start, and goal as PNG."""
        if not SAVE_TRIAL_MAP:
            return

        output_dir = os.path.expanduser(TRIAL_MAP_OUTPUT_DIR)
        os.makedirs(output_dir, exist_ok=True)

        filename = time.strftime("path_planner_trial_%Y%m%d_%H%M%S.png")
        output_path = os.path.join(output_dir, filename)

        scale = max(1, int(TRIAL_MAP_PIXEL_SCALE))
        width = self.grid.cols * scale
        height = self.grid.rows * scale
        pixels = bytearray(width * height * 3)

        free_color = (245, 245, 245)
        obstacle_color = (35, 35, 35)

        def paint_grid_cell(row, col, color):
            if row < 0 or row >= self.grid.rows:
                return

            if col < 0 or col >= self.grid.cols:
                return

            for py in range(row * scale, (row + 1) * scale):
                for px in range(col * scale, (col + 1) * scale):
                    index = (py * width + px) * 3
                    pixels[index] = color[0]
                    pixels[index + 1] = color[1]
                    pixels[index + 2] = color[2]

        def paint_circle(cell, radius_cells, color):
            if cell is None:
                return

            center_row, center_col = cell

            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    if math.sqrt(dr ** 2 + dc ** 2) <= radius_cells:
                        paint_grid_cell(center_row + dr, center_col + dc, color)

        def paint_line(start_cell, end_cell, color):
            if start_cell is None or end_cell is None:
                return

            row0, col0 = start_cell
            row1, col1 = end_cell

            d_col = abs(col1 - col0)
            d_row = -abs(row1 - row0)
            step_col = 1 if col0 < col1 else -1
            step_row = 1 if row0 < row1 else -1
            error = d_col + d_row

            while True:
                paint_circle((row0, col0), 1, color)

                if row0 == row1 and col0 == col1:
                    break

                error2 = 2 * error

                if error2 >= d_row:
                    error += d_row
                    col0 += step_col

                if error2 <= d_col:
                    error += d_col
                    row0 += step_row

        def paint_world_polyline(points, color):
            if len(points) == 0:
                return

            previous_cell = self.grid.world_to_grid(points[0][0], points[0][1])
            paint_circle(previous_cell, 2, color)

            for point in points[1:]:
                current_cell = self.grid.world_to_grid(point[0], point[1])
                paint_line(previous_cell, current_cell, color)
                previous_cell = current_cell

        for row in range(self.grid.rows):
            for col in range(self.grid.cols):
                if self.grid.grid[row][col] == 1:
                    paint_grid_cell(row, col, obstacle_color)
                else:
                    paint_grid_cell(row, col, free_color)

        paint_world_polyline(self.path, (50, 110, 235))
        paint_world_polyline(self.robot_trail, (35, 170, 85))

        paint_circle(self.grid.world_to_grid(0.0, 0.0), 4, (125, 70, 180))
        paint_circle(self.grid.world_to_grid(self.goal_x, self.goal_y), 4, (220, 35, 35))
        paint_circle(self.grid.world_to_grid(self.robot_x, self.robot_y), 4, (245, 145, 35))

        self.write_png(output_path, width, height, pixels)
        self.get_logger().info(f"Saved trial map: {output_path}")

    def write_png(self, output_path, width, height, pixels):
        """Write RGB pixels to a PNG file using only the Python standard library."""
        def png_chunk(chunk_type, data):
            chunk = chunk_type + data
            return (
                struct.pack(">I", len(data)) +
                chunk +
                struct.pack(">I", zlib.crc32(chunk) & 0xffffffff)
            )

        raw_rows = bytearray()
        row_bytes = width * 3

        for row in range(height):
            raw_rows.append(0)
            start = row * row_bytes
            raw_rows.extend(pixels[start:start + row_bytes])

        png_data = bytearray()
        png_data.extend(b"\x89PNG\r\n\x1a\n")
        png_data.extend(png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
        png_data.extend(png_chunk(b"IDAT", zlib.compress(bytes(raw_rows), 9)))
        png_data.extend(png_chunk(b"IEND", b""))

        with open(output_path, "wb") as png_file:
            png_file.write(png_data)

    def finish_goal(self):
        """Stop the robot and shut down the node after the target is reached."""
        if self.goal_reached:
            return

        self.get_logger().info("Goal reached!")

        self.goal_reached = True
        self.recovery_mode = False
        self.need_replan = False
        self.replan_straight_mode = False

        for _ in range(10):
            self.stop_robot()

        try:
            self.save_trial_map()
        except Exception as exc:
            self.get_logger().warn(f"Could not save trial map: {exc}")

        self.path = []
        self.path_index = 0

        self.shutdown_requested = True
        self.get_logger().info("Goal reached. Shutting down path planner.")

        if rclpy.ok():
            rclpy.shutdown()

    def stop_for_manual_shutdown(self):
        """Stop the robot when the user exits with Ctrl+C."""
        # Ctrl+C should stop the robot before ROS shuts down the publisher.
        self.goal_reached = True
        self.recovery_mode = False
        self.path = []
        self.path_index = 0
        self.need_replan = False
        self.replan_straight_mode = False

        for _ in range(50):
            self.stop_robot()
            time.sleep(0.03)


def main(args=None):
    """Start the navigator node and stop the robot cleanly on exit."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = InformedRRTNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Ctrl+C detected. Sending zero velocity before shutdown...")
    except ExternalShutdownException:
        pass

    finally:
        try:
            node.timer.cancel()
        except Exception:
            pass

        if not node.shutdown_requested:
            node.stop_for_manual_shutdown()
            print("Ctrl+C/manual shutdown: robot stopped at current position.")

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
