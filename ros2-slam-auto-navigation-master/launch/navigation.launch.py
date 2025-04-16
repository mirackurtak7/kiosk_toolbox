from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Sim time argümanı
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time.'
    )

    # Nav2 parametre dosyası argümanı
    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_slam_auto_navigation'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for nav2'
    )

    # Nav2 bringup launch'ını dahil et
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # RViz2 başlatılması
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
        nav2_params_arg,
        nav2_bringup_launch,
        rviz_node
    ])
