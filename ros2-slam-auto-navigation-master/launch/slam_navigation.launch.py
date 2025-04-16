from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/root/docker/map.yaml',
        description='Full path to map file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_slam_auto_navigation'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Nav2 config YAML'
    )

    # Nav2 bringup_launch.py çağırılıyor
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'slam': 'false',
            'localization': 'true'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        nav2_bringup_launch,
        rviz_node
    ])
