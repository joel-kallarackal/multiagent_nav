from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # Path to the parameter file inside your package
    params_file = os.path.join(
        get_package_share_directory('turtle_nav'),
        'params',
        'mapping_params.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=params_file, description='SLAM toolbox params file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        
        # Launch the slam_toolbox node with the parameters loaded from the YAML file
        launch_ros.actions.Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],  # Load the params file here
            remappings=[('/scan', '/robot2/scan'),
                        ('/odom', '/robot2/odom'),
                        ('/tf', '/robot2/tf'),
                        ('/tf_static', '/robot2/tf_static'),],  # Adjust if needed for your laser scan topic
            arguments=['--ros-args', '--param', 'queue_size:=100']
        ),
    ])
