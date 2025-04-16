#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gui = LaunchConfiguration('gui', default='True')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    urdf_file_name = 'dolly.urdf'
    urdf = os.path.join(
        get_package_share_directory('ali_rob'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value=gui,
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Flag to enable use_sim_time'
        ),

        LogInfo(
            condition=IfCondition(gui),
            msg=['Launching with GUI enabled']
        ),
        LogInfo(
            condition=UnlessCondition(gui),
            msg=['Launching without GUI']
        ),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time}
            ],
            respawn=True,
            respawn_delay=1.0
        ),

        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui),
            respawn=True,
            respawn_delay=1.0
        ),

        # Joint State Publisher GUI Node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            respawn=True,
            respawn_delay=1.0
        )
    ])
