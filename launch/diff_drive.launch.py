# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('robo_diferenciado')
    pkg_project_gazebo = get_package_share_directory('robo_diferenciado')
    pkg_project_description = get_package_share_directory('robo_diferenciado')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_project_description, 'urdf', 'dif_base.urdf.xacro')
    control_config_path = os.path.join(pkg_project_description, 'config', 'diff_drive_control.yaml')

    # Load the SDF file from 'description' package

    # Wheel radius and separation are taken from the controller config YAML
    with open(control_config_path) as f:
        params = yaml.safe_load(f)

    controller_params = params['diff_drive_controller']['ros__parameters']

    # Access YAML parameters:
    wheel_radius = controller_params['wheel_radius']
    wheel_separation = controller_params['wheel_separation']

    print('Wheel radius from YAML:', wheel_radius)
    print('Wheel separation from YAML:', wheel_separation)

    robot_description = Command(['xacro ', urdf_path, ' ',
                                'wheel_radius:=', str(wheel_radius), ' ',
                                'wheel_separation:=', str(wheel_separation/2), ' ',
                                'wheel_width:=0.05 ',
                                'wheel_mass:=1.0 ',
                                'caster_radius:=0.05 ',
                                'caster_mass:=0.5 ',
                                'caster_offset_x:=0.15 ',
                                'chassis_mass:=5.0 ',
                                'chassis_length:=0.4 ',
                                'chassis_width:=0.3 ',
                                'chassis_height:=0.1 '])


    ############# Gazebo 2 ROS2 Launch

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive.sdf'
        ])]}.items(),
    )

    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robo_diferenciado',
            '-topic', 'robot_description',
            # '-x', str(pose_params['x']),
            # '-y', str(pose_params['y']),
            # '-z', str(0.1),
            # '-R', str(pose_params['roll']),
            # '-P', str(pose_params['pitch']),
            # '-Y', str(pose_params['yaw']),
        ],
        output='screen'
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': True}]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    odom_node = Node(
        package='robo_diferenciado',
        executable='odom',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'wheel_radius': wheel_radius},
            {'wheel_separation': wheel_separation}
        ]
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    map_file = os.path.join(pkg_project_description, 'map', 'map.yaml')

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_file
        }]
    )
    lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server"]
        }]
    )

    return LaunchDescription([
        gz_sim,
        gazebo_spawn_robot,
        robot_state_publisher,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        rviz,
        bridge,
        odom_node,
        diff_drive_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        map_server_node,
        lifecycle_node,
    ])
