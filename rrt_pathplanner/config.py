# Default target in the robot's starting coordinate frame.
# x is forward from the robot's starting heading, y is left.
# These can be changed from the terminal:
# ros2 run path_planner path_planner --ros-args -p goal_x:=2.0 -p goal_y:=1.0
DEFAULT_GOAL_X = 1.5
DEFAULT_GOAL_Y = 0.0


# Local map size around the robot's starting point, in meters.
# Increase these if the target is outside the map.
MAP_WIDTH_M = 15.0
MAP_HEIGHT_M = 15.0

# Smaller resolution gives a more detailed map, but planning takes longer.
GRID_RESOLUTION = 0.1

# Informed RRT* planner settings.
# The planner still uses the LiDAR-built occupancy grid, but samples paths instead
# of expanding every grid cell like Dijkstra.
RRT_MAX_ITERATIONS = 2000

# One tree extension length, in meters.
RRT_STEP_SIZE = 0.25

# Probability of sampling the goal before a first path is found.
RRT_GOAL_SAMPLE_RATE = 0.15

# If a tree node is within this distance of the goal, try connecting to the goal.
RRT_GOAL_CONNECT_DISTANCE = 0.35

# Distance between collision checks along a proposed edge, in meters.
RRT_COLLISION_CHECK_STEP = 0.05

# RRT* rewiring neighborhood radius, in meters.
RRT_REWIRE_RADIUS = 0.70

# Number of random shortcut attempts used to simplify the final path.
RRT_SHORTCUT_ATTEMPTS = 120


# Extra safety space around each obstacle. Increase this if the robot gets too close.
OBSTACLE_INFLATION_RADIUS = 0.16

# Safety radius used when another robot's final goal is reserved in this
# robot's local occupancy map before multibot navigation starts.
GOAL_POINT_INFLATION_RADIUS = 0.25

# LiDAR readings farther than this are ignored for obstacle marking.
MAX_LIDAR_OBSTACLE_RANGE = 3.0

# If an obstacle is closer than this in front, the robot enters recovery.
FRONT_OBSTACLE_STOP_DISTANCE = 0.3

# Narrow straight-ahead danger zone. Obstacles here are treated as true front
# blockers even if the wider front cone also includes side-wall readings.
FRONT_HARD_STOP_ANGLE_DEG = 15.0

# Recovery is allowed to ignore a wide-front obstacle when the command is only
# creeping/turning away from it. Faster forward motion still triggers recovery.
RECOVERY_FORWARD_SPEED_TRIGGER = 0.03
RECOVERY_TURN_TOWARD_ANGULAR_TRIGGER = 0.05

# Side clearance threshold for smooth turning near parallel walls.
SIDE_CLEARANCE_TRIGGER_DISTANCE = 0.265
SIDE_CLEARANCE_ACTIVATION_ANGLE = 0.10
SIDE_CLEARANCE_MIN_FORWARD_SPEED = 0.02
SIDE_CLEARANCE_MAX_LINEAR_SPEED = 0.04
SIDE_CLEARANCE_MAX_ANGULAR_SPEED = 0.22
SIDE_CLEARANCE_SIDE_SECTOR_HALF_ANGLE = 50.0

# If the current path becomes blocked within this distance, replan before recovery.
PATH_REPLAN_LOOKAHEAD_DISTANCE = 4.0

# After an early replan, wait this many scans before triggering another one.
PATH_REPLAN_COOLDOWN_SCANS = 15

# While a blocked original path is being replanned, keep moving forward slowly
# instead of stopping or following the blocked path.
REPLAN_STRAIGHT_SPEED = 0.08

# Half-angle of the front danger zone, in radians.
# 0.785 rad is 45 degrees on each side, 90 degrees total.
FRONT_DETECTION_ANGLE = 0.785

# Do not back up during recovery if the rear sector is closer than this.
RECOVERY_REAR_CLEAR_DISTANCE = 0.1

# Recovery follows the robot's recent trail backward instead of turning in place.
# Negative speed means backing up.
RECOVERY_BACKUP_SPEED = -0.10

# Maximum recovery duration. During initial startup only, recovery is shortened
# to the number of normal movement ticks accumulated so far. After the robot has
# moved for this many ticks, every recovery uses this fixed duration.
# Timer ticks are based on the 0.1 second control loop.
RECOVERY_BACKUP_TICKS = 20

# How far back in the recorded trail recovery should aim.
RECOVERY_BACKTRACK_LOOKBACK_POINTS = 8
RECOVERY_BACKTRACK_ANGULAR_GAIN = 1.5
RECOVERY_BACKTRACK_MAX_ANGULAR_SPEED = 0.5

# Motion limits for the path follower.
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 0.8

# Distance from a waypoint where it counts as reached.
WAYPOINT_TOLERANCE = 0.12

# Controller gains. Higher values react faster, but can make motion less smooth.
K_LINEAR = 0.8
K_ANGULAR = 1.5

# If the angle error is larger than this, rotate in place before driving forward.
ROTATE_FIRST_ANGLE = 0.40

# Keep every Nth path point. Use 1 for the full Dijkstra path.
PATH_DOWNSAMPLE_STEP = 1

# Distance from the final target where the program prints "Goal reached".
GOAL_TOLERANCE = 0.1

# Save a PNG map when the robot reaches the goal.
SAVE_TRIAL_MAP = True
TRIAL_MAP_OUTPUT_DIR = "path_planner_maps"
TRIAL_MAP_PIXEL_SCALE = 2


# Multi-robot coordination.
# Keep false for normal single-robot behavior. In ROS, this can be changed with:
# ros2 run rrt_pathplanner rrt_pathplanner --ros-args -p multibot:=1
MULTIBOT_DEFAULT = 0

# Each robot should override robot_id when multibot is enabled.
DEFAULT_ROBOT_ID = "robot"

# Approximate robot body radius used by the central traffic coordinator.
ROBOT_RADIUS = 0.18

# Transform this robot's local start frame into the shared multibot traffic frame.
# If robot A starts at the room origin and robot B starts 1 m to its left, run
# robot B with -p shared_origin_y:=1.0.
MULTIBOT_SHARED_ORIGIN_X = 0.0
MULTIBOT_SHARED_ORIGIN_Y = 0.0
MULTIBOT_SHARED_ORIGIN_YAW = 0.0

# Shared coordination topics. Bridge these topics between ROS_DOMAIN_IDs when
# robots are isolated in different domains.
MULTIBOT_STATE_TOPIC = "/multibot/robot_state"
MULTIBOT_START_TOPIC = "/multibot/start"
MULTIBOT_CONTROL_TOPIC = "/multibot/control"

# How often each robot publishes its state to the coordinator.
MULTIBOT_STATE_PUBLISH_PERIOD = 0.20

# Central ATP-style traffic grid. The local planner still uses GRID_RESOLUTION.
TRAFFIC_GRID_RESOLUTION = 0.35

# Yellow zone expands around each robot's red footprint by this many traffic cells.
TRAFFIC_YELLOW_RADIUS_CELLS = 0.35

# Number of upcoming traffic cells checked for "about to enter red zone".
TRAFFIC_LOOKAHEAD_CELLS = 2

# Minimum center-to-center robot spacing in yellow-zone checks is:
# radius_a + radius_b + MULTIBOT_SAFETY_MARGIN.
MULTIBOT_SAFETY_MARGIN = 0.15

# Extra distance before releasing a robot that was stopped near another robot.
MULTIBOT_RESUME_HYSTERESIS = 0.15

# Robot states older than this are ignored by the central coordinator.
MULTIBOT_STATE_TIMEOUT = 1.50

# Robot stops if central control messages stop arriving after START.
MULTIBOT_CONTROL_TIMEOUT = 1.00

# Coordinator control loop period.
MULTIBOT_COORDINATOR_PERIOD = 0.10

# Slow command cap used by the coordinator when needed.
MULTIBOT_SLOW_LINEAR_SPEED = 0.05
