import os
import launch
import pathlib
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_epuck')
    package_ekf = get_package_share_directory('robot_localization')
    package_mcl = get_package_share_directory('ros_analise')
    use_rviz = LaunchConfiguration('rviz', default=True)
    synchronization = LaunchConfiguration('synchronization', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world = LaunchConfiguration('world', default='rats_life_benchmark_gps.wbt')
    robot_description = pathlib.Path(os.path.join(get_package_share_directory('ros_analise'), 'resource', 'epuck_webots.urdf')).read_text()
    mission_time = LaunchConfiguration('mission_time', default=5)
    rviz_config = os.path.join(get_package_share_directory('ros_analise'), 'resource', 'config-ekf.rviz')

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch/robot_launch_gps.py')
        ),
        launch_arguments={
            'synchronization': synchronization,
            'world': world,
            'use_sim_time': use_sim_time
        }.items()
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_mcl, 'ekf.launch.py')
        ),
        launch_arguments={
            'synchronization': synchronization,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        webots_launch,
        localization_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['--display-config=' + rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=launch.conditions.IfCondition(use_rviz)
        ),
        Node(
            package='ros_analise',
            executable='monte_carlo_localizer_mapa2',
            output='log'
        ),
    ])
