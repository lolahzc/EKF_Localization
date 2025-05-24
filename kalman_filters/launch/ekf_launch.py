from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_path = os.path.expanduser('~/ar_ws/src/EKF_Localization/moving_point/rviz/default.rviz')    

    return LaunchDescription([
        # Lanzamos el nodo simulator
        Node(
            package='moving_point',
            executable='random_point_node',
            name='simulator',
            parameters=[{
                'odom_noise_variance': 0.01,  # Reducir de 0.05
                'gps_noise_variance': 0.1    # Mantener o reducir
            }]
        ),
        # Lanzamos el nodo static_map
        Node(
            package='moving_point',
            executable='static_map_node',
            name='static_map'
        ),
        # Lanzamos el nodo del filtro EKF
        Node(
            package='kalman_filters',  # Paquete donde está el nodo ekf_node
            executable='ekf_node',     # Ejecutable que contiene el nodo
            name='ekf_filter'
        ),
        # Ejecutamos RViz con el archivo de configuración
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])
