from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_recorder',
            executable='waypoint_recorder_node',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'button_index': 0,  # A button
                'output_file': '/tmp/rack_waypoints.json',
                'pose_topic': '/tracked_pose',  # Cartographer
                'use_odom_fallback': True,
            }]
        )
    ])
