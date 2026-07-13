# Informed_RRT-_for_Turtlebot3_26Summer

## Single Robot Mode

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select rrt_pathplanner
source install/setup.bash
```

Run one robot normally:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run rrt_pathplanner rrt_pathplanner --ros-args \
  -p goal_x:=1.5 \
  -p goal_y:=0.0
```

## Multibot Control Manual

In multibot mode, each robot still runs its own local RRT planner. Before
starting, the coordinator sends every robot's final goal to the other robots.
Those goals are reserved as inflated obstacles in each local map. During
navigation, the coordinator also sends safety commands:

```text
RUN
SLOW
HOLD
EMERGENCY_STOP
```

Each robot waits for the central coordinator before moving. After you type `Y`
in the central terminal, the robots start navigating to their own goals.

## Gazebo Two-Robot Test

Do not use the stock `turtlebot3_gazebo multi_robot.launch.py` if it fails with
permission denied in `/opt/ros/humble`. This package includes a two-Waffle launch
file that writes temporary SDF files to `/tmp` instead.

Build first:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select rrt_pathplanner --symlink-install
source install/setup.bash
```

Terminal 1, start Gazebo with two namespaced Waffle robots:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch rrt_pathplanner two_waffles.launch.py \
  tb3_1_x:=0.0 \
  tb3_1_y:=0.0 \
  tb3_1_yaw:=0.0 \
  tb3_2_x:=-0.5 \
  tb3_2_y:=0.0 \
  tb3_2_yaw:=0.0
```

Check the simulated robot topics:

```bash
ros2 topic list | grep TB3
```

You should see topics like:

```text
/TB3_1/scan
/TB3_1/odom
/TB3_1/cmd_vel
/TB3_2/scan
/TB3_2/odom
/TB3_2/cmd_vel
```

Terminal 2, planner for `TB3_1`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 run rrt_pathplanner rrt_pathplanner --ros-args \
  -r __node:=rrt_TB3_1 \
  -r /scan:=/TB3_1/scan \
  -r /odom:=/TB3_1/odom \
  -r /cmd_vel:=/TB3_1/cmd_vel \
  -r /goal_marker:=/TB3_1/goal_marker \
  -p multibot:=1 \
  -p robot_id:=TB3_1 \
  -p goal_x:=2.0 \
  -p goal_y:=0.0 \
  -p robot_radius:=0.18 \
  -p goal_point_inflation_radius:=0.46 \
  -p shared_origin_x:=0.0 \
  -p shared_origin_y:=0.0 \
  -p shared_origin_yaw:=0.0
```

Terminal 3, planner for `TB3_2`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 run rrt_pathplanner rrt_pathplanner --ros-args \
  -r __node:=rrt_TB3_2 \
  -r /scan:=/TB3_2/scan \
  -r /odom:=/TB3_2/odom \
  -r /cmd_vel:=/TB3_2/cmd_vel \
  -r /goal_marker:=/TB3_2/goal_marker \
  -p multibot:=1 \
  -p robot_id:=TB3_2 \
  -p goal_x:=4.0 \
  -p goal_y:=0.0 \
  -p robot_radius:=0.18 \
  -p goal_point_inflation_radius:=0.46 \
  -p shared_origin_x:=-0.5 \
  -p shared_origin_y:=0.0 \
  -p shared_origin_yaw:=0.0
```

Terminal 4, central coordinator:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 run rrt_pathplanner multibot_coordinator --ros-args \
  -p robot_ids:="TB3_1,TB3_2" \
  -p safety_margin:=0.10
```

Wait until the coordinator prints:

```text
Initial Setup is Ready, Goal Point Marked, Press Y to Run
```

Then type `Y`. Gazebo uses one ROS domain here, so no `domain_bridge` is needed
for the simulation test.

## Coordination Topics

Only these topics need to be shared between ROS domains:

```text
/multibot/robot_state
/multibot/start
/multibot/control
```

Robot LiDAR, odom, and cmd_vel topics stay inside each robot's own domain.

## Install Domain Bridge

If `domain_bridge` is not installed:

```bash
sudo apt install ros-$ROS_DISTRO-domain-bridge
```

Check that it is available:

```bash
ros2 pkg executables domain_bridge
```

## Build Package

Run once before starting the robots:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select rrt_pathplanner
source install/setup.bash
```

## Create Bridge Config

Create a config file for bridging domain `30` and domain `31`:

```bash
cat > /tmp/multibot_bridge.yaml <<'YAML'
name: multibot_bridge
from_domain: 30
to_domain: 31
topics:
  /multibot/robot_state:
    type: std_msgs/msg/String
    bidirectional: true
  /multibot/start:
    type: std_msgs/msg/String
    bidirectional: true
  /multibot/control:
    type: std_msgs/msg/String
    bidirectional: true
YAML
```

## Terminal 1: Robot In Domain 30

Example for robot `tb3_30`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30

ros2 run rrt_pathplanner rrt_pathplanner --ros-args \
  -p multibot:=1 \
  -p robot_id:=tb3_30 \
  -p goal_x:=2.0 \
  -p goal_y:=1.0 \
  -p robot_radius:=0.18 \
  -p shared_origin_x:=0.0 \
  -p shared_origin_y:=0.0 \
  -p shared_origin_yaw:=0.0
```

## Terminal 2: Robot In Domain 31

Example for robot `tb3_31`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=31

ros2 run rrt_pathplanner rrt_pathplanner --ros-args \
  -p multibot:=1 \
  -p robot_id:=tb3_31 \
  -p goal_x:=1.5 \
  -p goal_y:=-1.0 \
  -p robot_radius:=0.18 \
  -p shared_origin_x:=1.0 \
  -p shared_origin_y:=0.0 \
  -p shared_origin_yaw:=0.0
```

## Terminal 3: Domain Bridge

Run the bridge after both robot terminals are ready:

```bash
source /opt/ros/humble/setup.bash
ros2 run domain_bridge domain_bridge /tmp/multibot_bridge.yaml
```

## Terminal 4: Central Coordinator

Run the central traffic coordinator in one bridged domain, for example domain
`30`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30

ros2 run rrt_pathplanner multibot_coordinator --ros-args \
  -p robot_ids:="tb3_30,tb3_31"
```

When the coordinator prints the prompt, type:

```text
Y
```

Both robots should then start moving to their own goals.

## Important Parameters

`multibot`

```text
0 = normal single robot mode
1 = wait for central coordinator and obey multibot safety commands
```

`robot_id`

```text
Unique name for this robot, for example tb3_30 or tb3_31.
```

`goal_x`, `goal_y`

```text
The local goal for that robot, in that robot's own starting coordinate frame.
x is forward from that robot's starting heading.
y is left from that robot's starting heading.
```

`robot_radius`

```text
Approximate physical radius of the robot in meters.
The coordinator uses this for red/yellow zone safety checks.
```

`shared_origin_x`, `shared_origin_y`, `shared_origin_yaw`

```text
Place this robot's local start frame into one shared traffic frame.
This is required because each robot's odom frame starts at its own origin.
```

Example:

```text
Robot 30 starts at the shared room origin:
shared_origin_x = 0.0
shared_origin_y = 0.0
shared_origin_yaw = 0.0

Robot 31 starts 1.0 m to the right in the shared room frame:
shared_origin_x = 0.0
shared_origin_y = -1.0
shared_origin_yaw = 0.0
```

## How The Avoidance Works

The central coordinator uses a coarse traffic grid, separate from the local RRT
grid:

```text
local planner grid: 0.1 m
central traffic grid: 0.5 m
```

For each robot:

```text
red zone = cells occupied by the robot footprint
yellow zone = neighbor cells around the red zone
```

Rules:

```text
If another robot is about to enter a red zone:
  HOLD that robot.

If two robots overlap in red zone:
  EMERGENCY_STOP the lower-priority robot.

If a robot enters a yellow zone:
  check real center-to-center distance.

If distance <= radius_a + radius_b + safety_margin:
  HOLD the lower-priority robot.

If distance is safe:
  RUN as usual.
```

The central coordinator never edits the local route. It only gates velocity.

## Add More Robots

For a third robot, run another robot terminal with a new domain and ID:

```bash
export ROS_DOMAIN_ID=32

ros2 run rrt_pathplanner rrt_pathplanner --ros-args \
  -p multibot:=1 \
  -p robot_id:=tb3_32 \
  -p goal_x:=0.0 \
  -p goal_y:=2.0 \
  -p robot_radius:=0.18 \
  -p shared_origin_x:=0.0 \
  -p shared_origin_y:=1.0 \
  -p shared_origin_yaw:=0.0
```

Then update the coordinator robot list:

```bash
ros2 run rrt_pathplanner multibot_coordinator --ros-args \
  -p robot_ids:="tb3_30,tb3_31,tb3_32"
```

You also need to bridge the new robot's domain so it can share the
`/multibot/*` topics with the coordinator domain.

## Quick Debug Commands

List multibot topics in the current domain:

```bash
ros2 topic list | grep multibot
```

Watch robot states:

```bash
ros2 topic echo /multibot/robot_state
```

Watch central commands:

```bash
ros2 topic echo /multibot/control
```

Manually send START for testing:

```bash
ros2 topic pub --once /multibot/start std_msgs/msg/String \
  "{data: '{\"command\":\"START\"}'}"
```

Manually HOLD one robot:

```bash
ros2 topic pub --once /multibot/control std_msgs/msg/String \
  "{data: '{\"robot_id\":\"tb3_30\",\"command\":\"HOLD\",\"reason\":\"manual_test\"}'}"
```

Manually RUN one robot:

```bash
ros2 topic pub --once /multibot/control std_msgs/msg/String \
  "{data: '{\"robot_id\":\"tb3_30\",\"command\":\"RUN\",\"reason\":\"manual_test\"}'}"
```
