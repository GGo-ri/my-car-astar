# src/my_bot/launch/tb3_slam_real.launch.py
# 실제 TurtleBot3용 SLAM (PC에서 실행). Gazebo 없음, use_sim_time 기본값 false.
# 로봇(라즈베리파이)에서 먼저 `ros2 launch turtlebot3_bringup robot.launch.py`를 띄워 둘 것.
# robot_state_publisher는 로봇 쪽 bringup이 띄우므로 여기서는 띄우지 않는다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_cartographer'),
        'rviz', 'tb3_cartographer.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='실제 로봇이므로 false'),

        slam_launch,
        rviz_node
    ])
