from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument

planner_params_file = os.path.join(
        get_package_share_directory('turtle_nav'),
        'params',
        'planner_server.yaml'
    )

controller_params_file = os.path.join(
        get_package_share_directory('turtle_nav'),
        'params',
        'controller_server.yaml'
    )

def generate_launch_description():
    return LaunchDescription([
        # Planner Server (Global Planner + Global Costmap)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_params_file],
        ),

        # Controller Server (Local Planner + Local Costmap)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_params_file],
        ),

        # Lifecycle Manager (to bring up planner & controller)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server', 'controller_server']
            }],
        )
    ])
