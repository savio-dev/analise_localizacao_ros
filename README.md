## Analise de Métodos de Localização com o ROS

## Descrição
Foram implementados os métodos de localização de Monte Carlo, AMCL, Fusão de sensores e combinação de AMCL e fusão de sensores utilizando o ROS2 e o Webots

## Compilação
Configure um ambiente de trabalho do ROS e instale o pacote da seguinte forma: 
```commandline
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --packages-select ros_analise
. install/setup.bash
```

Depois de compilar e instalar o pacote, você pode iniciar a simulação.
### Execução
```commandline
ros2 launch ros_analise mcl_launch.py
ros2 launch ros_analise mcl_launch_mapa2.py
ros2 launch ros_analise amcl_launch_nav2.py
ros2 launch ros_analise amcl_launch_nav2_mapa2.py
ros2 launch ros_analise ekf_launch.py
ros2 launch ros_analise ekf_launch_mapa2.py
ros2 launch ros_analise amcl_ekf_launch.py
ros2 launch ros_analise amcl_ekf_launch_mapa2.py
```

## Dependencias
Webots ROS2 package
