from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('moving_point')
    rviz_config_path = os.path.expanduser('~/adrpro/src/moving_point/rviz/default.rviz')    
    
    return LaunchDescription([
        Node(
            package='moving_point',
            executable='random_point_node',
            name='moving_point',
            parameters=[{
                'odom_noise_variance': 0.05,
                'gps_noise_variance': 0.2,
                'simulate_irregularities': False
            }]
        ),

        Node(
            package='moving_point',
            executable='static_map_node',
            name='static_map_node',
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])