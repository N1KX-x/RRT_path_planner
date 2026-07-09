#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def create_namespaced_sdf(model, namespace):
    """Create a namespaced TurtleBot3 SDF in /tmp and return its path."""
    gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    model_folder = 'turtlebot3_' + model
    source_path = os.path.join(gazebo_share, 'models', model_folder, 'model.sdf')
    output_path = os.path.join('/tmp', f'rrt_pathplanner_{namespace}.sdf')

    tree = ET.parse(source_path)
    root = tree.getroot()

    for odom_frame_tag in root.iter('odometry_frame'):
        odom_frame_tag.text = f'{namespace}/odom'

    for base_frame_tag in root.iter('robot_base_frame'):
        base_frame_tag.text = f'{namespace}/base_footprint'

    for scan_frame_tag in root.iter('frame_name'):
        scan_frame_tag.text = f'{namespace}/base_scan'

    sdf_text = '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding='unicode')

    with open(output_path, 'w') as sdf_file:
        sdf_file.write(sdf_text)

    return output_path


def spawn_robot(namespace, entity_name, sdf_path, x_pose, y_pose, yaw):
    """Return launch actions for one namespaced Gazebo TurtleBot3."""
    gazebo_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch'
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_prefix': namespace,
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', entity_name,
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw,
            '-robot_namespace', namespace,
        ],
        output='screen'
    )

    return GroupAction([
        PushRosNamespace(namespace),
        robot_state_publisher,
        spawn_entity,
    ])


def generate_launch_description():
    """Launch Gazebo with two namespaced TurtleBot3 Waffle robots."""
    model = 'waffle'
    tb3_1_sdf = create_namespaced_sdf(model, 'TB3_1')
    tb3_2_sdf = create_namespaced_sdf(model, 'TB3_2')

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    world_path = os.path.join(
        turtlebot3_gazebo_share,
        'worlds',
        'turtlebot3_world.world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    tb3_1_x = LaunchConfiguration('tb3_1_x', default='-2.0')
    tb3_1_y = LaunchConfiguration('tb3_1_y', default='-0.5')
    tb3_1_yaw = LaunchConfiguration('tb3_1_yaw', default='0.0')
    tb3_2_x = LaunchConfiguration('tb3_2_x', default='0.5')
    tb3_2_y = LaunchConfiguration('tb3_2_y', default='-2.0')
    tb3_2_yaw = LaunchConfiguration('tb3_2_yaw', default='0.0')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: [
                os.remove(path)
                for path in (tb3_1_sdf, tb3_2_sdf)
                if os.path.exists(path)
            ]
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('tb3_1_x', default_value='-2.0'),
        DeclareLaunchArgument('tb3_1_y', default_value='-0.5'),
        DeclareLaunchArgument('tb3_1_yaw', default_value='0.0'),
        DeclareLaunchArgument('tb3_2_x', default_value='0.5'),
        DeclareLaunchArgument('tb3_2_y', default_value='-2.0'),
        DeclareLaunchArgument('tb3_2_yaw', default_value='0.0'),
        gzserver,
        gzclient,
        cleanup,
        spawn_robot('TB3_1', 'waffle_1', tb3_1_sdf, tb3_1_x, tb3_1_y, tb3_1_yaw),
        spawn_robot('TB3_2', 'waffle_2', tb3_2_sdf, tb3_2_x, tb3_2_y, tb3_2_yaw),
    ])
