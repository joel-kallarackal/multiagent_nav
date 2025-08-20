from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument

params_file = os.path.join(
        get_package_share_directory('turtle_nav'),
        'params',
        'local_costmap_params.yaml'
    )

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav2',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server']
            }]
        )
    ])

