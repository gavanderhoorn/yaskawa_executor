from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the node (e.g., debug, info, warn, error, fatal)'
        ),

        # Define the Node action
        Node(
            package='yaskawa_executor',
            executable='executeTraj_action',
            name='executeTraj_action',
            output='screen',
            parameters=[{
                'max_joint_msg_age': 0.05,
                'max_initial_deviation_rad': 0.02,
                'time_for_initial_adjustment': 1.0,
                'max_retries': 2000,
                'busy_wait_time': 0.005,
                'convergence_threshold': 0.01,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=[
                '--ros-args',
                '--log-level', LaunchConfiguration('log_level')
            ]
        )
    ])
