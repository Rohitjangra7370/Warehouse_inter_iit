from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_rover_scanning',
            executable='scanning_controller',
            name='scanning_controller',
            output='screen',
            parameters=[{
                'lateral_distance': 0.30,      # meters
                'lateral_speed': 0.05,         # m/s
                'lift_min_height': 0.45,       # meters
                'lift_max_height': 1.40,       # meters
                'lift_move_duration': 8.0,     # seconds
                'dwell_time_scan': 3.0,        # seconds
                'dwell_time_move': 1.0,        # seconds
            }]
        )
    ])
