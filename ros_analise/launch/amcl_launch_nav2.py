import os
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory('webots_ros2_epuck')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    synchronization = LaunchConfiguration('synchronization', default=True)
    world = LaunchConfiguration('world', default='epuck_world_gps.wbt')

    # Webots node
    launch_description_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'robot_launch.py')
            ),
            launch_arguments={
                'synchronization': synchronization,
                'use_sim_time': 'true',
                'world': world
            }.items()
        )
    )
    
    # Check if nav2_bringup is installed
    if 'nav2_bringup' in get_packages_with_prefixes():
        # Rviz node
        launch_description_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        )

        # Navigation
        nav2_map = os.path.join(package_dir, 'resource', 'epuck_world_map.yaml')

        launch_description_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                ),
                launch_arguments=[
                    ('map', nav2_map),
                    ('use_sim_time', use_sim_time),
                    ('params_file', os.path.join(package_dir, 'resource', 'nav2_params.yaml'))
                ],
            )
        )
        # Set initial position of the robot within the provided map.
        # The initial position can be also be set in RViz2 menu `2D Pose Estimate`.
        launch_description_nodes.append(ExecuteProcess(
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
        ))
    else:
        launch_description_nodes.append(LogInfo(msg='Navigation2 is not installed, navigation functionality is disabled'))

    # Launch descriptor
    return LaunchDescription(launch_description_nodes)
