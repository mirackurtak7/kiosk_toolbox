import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'ros2_slam_auto_navigation'
    urdf_package_name = 'ali_rob'

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(urdf_package_name),
                'launch',
                'rviz_.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Twist Mux
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # RPLIDAR
    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'view_rplidar_s1_launch.py'
            )
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': '256000',
            'frame_id': 'lidar'
        }.items()
    )

    # ZLAC Sürücü
    differential_drive_node = Node(
        package='zlac8015d_serial',
        executable='zlac_run',
        name='differential_drive',
        output='screen',
        respawn=True,
        respawn_delay=1.0
    )

    # Odometri
    odom_node = Node(
        package='odometry_calculator',
        executable='odometry_calculator',
        name='odom_calculate',
        output='screen',
        respawn=True,
        respawn_delay=1.0
    )

    # SLAM Toolbox (online async)
    slam_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml'
    )
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}]
    )

    # Localization (EKF)
    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')]
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        differential_drive_node,
        rplidar_node,
        odom_node,
        slam_toolbox_node,
        localization_node
    ])
