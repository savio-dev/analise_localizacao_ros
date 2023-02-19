import os
import launch
import pathlib
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_epuck')
    package_ekf = get_package_share_directory('robot_localization')
    package_ros_analise = get_package_share_directory('ros_analise')
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
            'use_sim_time': use_sim_time,
             'world' : world
        }.items()
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_ros_analise, 'ekf.amcl.launch.py')
        ),
        launch_arguments={
            'synchronization': synchronization,
            'use_sim_time': use_sim_time
        }.items()
    )
    
      # Check if nav2_bringup is installed
    if 'nav2_bringup' in get_packages_with_prefixes():
        nav2_map = os.path.join(package_dir, 'resource', 'map_rats_life.yaml')
        nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ), 
        launch_arguments=[
                    ('map', nav2_map),
                    ('use_sim_time', use_sim_time),
                    ('params_file', os.path.join(package_dir, 'resource', 'nav2_params.yaml'))
                ],
        )
        position_inicial = ExecuteProcess(
            cmd=[
                'ros2',
                'topic',
                'pub',
                '--once',
                '/initialpose',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                '{\
                "header": { "frame_id": "map" },\
                "pose": { "pose": {\
                    "position": { "x": 0.05, "y": 0.008, "z": 0.0 },\
                    "orientation": { "x": 0.0, "y": 0.0, "z": -1.0, "w": 3.14 }}\
                }\
            }'
            ]
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
        nav2_launch, 
        position_inicial,
    ])
