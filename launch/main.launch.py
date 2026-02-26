import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    astar_pkg_dir = get_package_share_directory('astar_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    

    map_file = LaunchConfiguration('map', default='/home/jgs/ros2_ws/slam_pkg/maps/world.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo 실행 
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Localization 실행
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
        }.items()
    )

    # astar_pure_yolo.py 노드 실행
    start_astar_yolo_cmd = Node(
        package='astar_pkg',
        executable='astar_pure_yolo', 
        name='integrated_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz 실행
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        start_gazebo_cmd,
        start_localization_cmd,
        start_astar_yolo_cmd,
        start_rviz_cmd
    ])