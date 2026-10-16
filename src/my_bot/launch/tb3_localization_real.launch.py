# src/my_bot/launch/tb3_localization_real.launch.py
# 실제 TurtleBot3용 위치 추정 (PC에서 실행): map_server + AMCL + RViz. Gazebo 없음, use_sim_time 기본값 false.
# 로봇(라즈베리파이)에서 먼저 `ros2 launch turtlebot3_bringup robot.launch.py`를 띄워 둘 것.
# 지도: 기본값 ~/maps/maze_map.yaml, 다른 지도는 `map:=<절대경로>.yaml`로 넘긴다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    map_file_path = os.path.expanduser('~/maps/maze_map.yaml')
    # TurtleBot3용으로 튜닝된 AMCL 파라미터 (nav2 범용 기본값 대신)
    params_file_path = os.path.join(turtlebot3_nav2_dir, 'param', 'humble', f'{turtlebot3_model}.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file_path,
            description='지도 yaml 파일의 절대 경로'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='실제 로봇이므로 false'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file_path,
            description='AMCL/map_server 파라미터 파일'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='false면 RViz를 띄우지 않음 (RViz를 따로 띄울 때)'),

        localization_launch,
        rviz_node
    ])
