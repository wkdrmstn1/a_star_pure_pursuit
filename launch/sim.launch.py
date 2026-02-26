import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 경로 및 설정 변수 정의
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # 맵 파일 경로 (사용자 요청 경로)
    map_file = LaunchConfiguration('map', default='/home/jgs/ros2_ws/slam_pkg/maps/world.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. 가제보 실행 (Gazebo)
    # 기본 터틀봇 월드를 엽니다. (다른 월드면 launch 파일명 변경 필요)
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 3. 내비게이션 실행 (Localization: Map Server + AMCL)
    # 요청하신 localization_launch.py를 실행합니다.
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml') # 기본 파라미터 사용
        }.items()
    )

    # 4. RViz 실행
    # Nav2 기본 설정이 적용된 RViz를 켭니다.
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 5. 실행 목록 리턴
    return LaunchDescription([
        start_gazebo_cmd,
        start_localization_cmd,
        start_rviz_cmd
    ])