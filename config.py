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
OBSTACLE_INFLATION_RADIUS = 0.30

# LiDAR readings farther than this are ignored for obstacle marking.
MAX_LIDAR_OBSTACLE_RANGE = 3.0

# If an obstacle is closer than this in front, the robot enters recovery.
FRONT_OBSTACLE_STOP_DISTANCE = 0.30

# If the current path becomes blocked within this distance, replan before recovery.
PATH_REPLAN_LOOKAHEAD_DISTANCE = 4.0

# After an early replan, wait this many scans before triggering another one.
PATH_REPLAN_COOLDOWN_SCANS = 15

# While a blocked original path is being replanned, keep moving forward slowly
# instead of stopping or following the blocked path.
REPLAN_STRAIGHT_SPEED = 0.08

# Half-angle of the front danger zone, in radians.
# 0.785 rad is 45 degrees on each side, 90 degrees total.
FRONT_DETECTION_ANGLE = 0.685

# Recovery is used when the robot gets blocked by an obstacle.
RECOVERY_TURN_SPEED = 0.5

# Recovery ends when the front LiDAR sector is clear by at least this distance.
RECOVERY_CLEAR_DISTANCE = 0.65

# Do not back up during recovery if the rear sector is closer than this.
RECOVERY_REAR_CLEAR_DISTANCE = 0.35

# Negative speed means backing up.
RECOVERY_BACKUP_SPEED = -0.10

# Timer ticks are based on the 0.1 second control loop.
RECOVERY_BACKUP_TICKS = 15
RECOVERY_MIN_TURN_TICKS = 12

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
