from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='/tmp/rack_waypoints.json',
        description='Path to waypoints JSON file'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto-start mission after delay'
    )
    
    start_delay_arg = DeclareLaunchArgument(
        'start_delay',
        default_value='10.0',
        description='Delay in seconds before auto-start'
    )
    
    return LaunchDescription([
        waypoints_file_arg,
        auto_start_arg,
        start_delay_arg,
        
        # Scanning Controller
        Node(
            package='warehouse_rover_scanning',
            executable='scanning_controller',
            name='scanning_controller',
            output='screen',
            parameters=[{
                'lateral_distance': 0.30,
                'lateral_speed': 0.05,
                'lift_min_height': 0.45,
                'lift_max_height': 1.40,
                'lift_move_duration': 8.0,
                'dwell_time_scan': 3.0,
                'dwell_time_move': 1.0,
            }]
        ),
        
        # Mission Executor
        Node(
            package='warehouse_rover_scanning',
            executable='mission_executor',
            name='mission_executor',
            output='screen',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'auto_start': LaunchConfiguration('auto_start'),
                'start_delay': LaunchConfiguration('start_delay'),
            }]
        ),
    ])
