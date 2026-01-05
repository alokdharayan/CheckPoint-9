from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Task 1: Pre-approach motion (WITH REMAP)
        Node(
            package='attach_shelf',
            executable='pre_approach',
            parameters=[
                {'obstacle': 0.3},
                {'degrees': -90.0}
            ],
            remappings=[
                ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
            ],
            output='screen'
        ),

        # Task 2: Service server
        Node(
            package='attach_shelf',
            executable='approach_service_server',
            output='screen'
        ),

        # Task 2: Service client trigger
        Node(
            package='attach_shelf',
            executable='pre_approach_v2',
            parameters=[
                {'final_approach': True}
            ],
            output='screen'
        )
    ])
