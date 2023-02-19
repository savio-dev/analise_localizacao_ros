from setuptools import setup

package_name = 'ros_analise'

data_files = []

data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
)
data_files.append(('share/' + package_name, [
      'launch/mcl_launch.py',
      'launch/amcl_launch_nav2.py',
      'launch/amcl_launch_nav2_mapa2.py',
      'launch/ekf_launch.py', 
      'launch/ekf.launch.py', 
      'launch/ekf.amcl.launch.py',
      'launch/ekf_launch_mapa2.py',
      'launch/mcl_launch_mapa2.py',
      'launch/ekf_amcl_launch.py',
      'launch/ekf_amcl_launch_mapa2.py',
      'launch/robot_launch.py']))
    
data_files.append(('share/' + package_name + '/worlds',[
    'worlds/epuck_world.wbt', 
    'worlds/epuck_world_gps.wbt', 
    'worlds/rats_life_benchmark_gps.wbt',
    'worlds/rats_life_benchmark.wbt' ]))
    
data_files.append(('share/' + package_name + '/resource', [
        'resource/motion_model.yaml',
        'resource/epuck_world_map.pgm',
        'resource/ekf.yaml', 
        'resource/ekf_amcl.yaml',
        'resource/cb_nav2_params.yaml',
        'resource/epuck_world_map.yaml',
        'resource/map_rats_life.pgm', 
        'resource/map_rats_life.yaml',
        'resource/ros2_control.yml', 
        'resource/configs.rviz', 
        'resource/config-ekf.rviz',
        'resource/default.rviz',
        'resource/sensor_model.yaml',
        'resource/mcl.yaml', 
        'resource/epuck_webots.urdf']))
        
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='savio',
    maintainer_email='saviooliveira769@gmail.com',
    description='Analise de Métodos de Localização com o ROS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iniciar_epuck = ros_analise.iniciar_epuck:main',
            'monte_carlo_localizer = ros_analise.monte_carlo_localizer:main',
            'monte_carlo_localizer_mapa2 = ros_analise.monte_carlo_localizer_mapa2:main',
        ],
    },
)
