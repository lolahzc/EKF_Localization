from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_path = os.path.expanduser('~/adrpro/src/EKF_Localization/moving_point/rviz/default.rviz')    

    return LaunchDescription([
        Node(
            package='moving_point',
            executable='random_point_node',
            name='simulator',
            parameters=[{
                'odom_noise_variance': 0.05,
                'gps_noise_variance': 0.2
            }]
        ),
        Node(
            package='moving_point',
            executable='static_map_node',
            name='static_map'
        ),
        Node(
            package='kalman_filters',
            executable='ekf_node',
            name='ekf_filter',
            parameters=[{
                'gps_noise': 2.0,           # Mayor valor = menos confianza en GPS
                'pos_process_noise': 0.005, # Menor valor = más confianza en modelo de posición
                'vel_process_noise': 0.0005 # Menor valor = más confianza en modelo de velocidad
        }]
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])