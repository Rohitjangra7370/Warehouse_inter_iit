#!/usr/bin/env python3
"""
Waypoint Recorder Node
Records robot positions when controller button is pressed during mapping
Saves waypoints for autonomous scanning mission
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import json
import os
from datetime import datetime
import math

class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Parameters
        self.declare_parameter('button_index', 0)  # A button (Xbox/PS controller)
        self.declare_parameter('output_file', '/tmp/rack_waypoints.json')
        self.declare_parameter('pose_topic', '/tracked_pose')  # Cartographer
        self.declare_parameter('use_odom_fallback', True)
        
        self.button_idx = self.get_parameter('button_index').value
        self.output_file = self.get_parameter('output_file').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.use_odom = self.get_parameter('use_odom_fallback').value
        
        # State
        self.current_pose = None
        self.last_button_state = 0
        self.waypoints = []
        self.rack_count = 0
        
        # Subscribe to pose (priority order: Cartographer > AMCL > Odom)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )
        
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        
        if self.use_odom:
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Load existing waypoints if file exists
        self.load_existing_waypoints()
        
        self.get_logger().info("╔════════════════════════════════════════╗")
        self.get_logger().info("║   🎮 WAYPOINT RECORDER READY          ║")
        self.get_logger().info("╚════════════════════════════════════════╝")
        self.get_logger().info(f"📌 Button: {self.button_idx} (A button)")
        self.get_logger().info(f"📡 Pose topic: {self.pose_topic}")
        self.get_logger().info(f"💾 Output: {self.output_file}")
        self.get_logger().info(f"📊 Current waypoints: {self.rack_count}")
        self.get_logger().info("")
        self.get_logger().info("🎯 Press button to save rack position!")
        self.get_logger().info("")
    
    def load_existing_waypoints(self):
        """Load existing waypoints from file"""
        if os.path.exists(self.output_file):
            try:
                with open(self.output_file, 'r') as f:
                    data = json.load(f)
                    self.waypoints = data.get('waypoints', [])
                    self.rack_count = len(self.waypoints)
                self.get_logger().info(f"📂 Loaded {self.rack_count} existing waypoints")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Could not load existing waypoints: {e}")
    
    def pose_callback(self, msg):
        """Update from Cartographer tracked_pose"""
        self.current_pose = msg.pose
    
    def amcl_callback(self, msg):
        """Fallback: update from AMCL"""
        if self.current_pose is None:
            self.current_pose = msg.pose.pose
    
    def odom_callback(self, msg):
        """Last resort: update from odometry"""
        if self.current_pose is None:
            self.current_pose = msg.pose.pose
    
    def joy_callback(self, msg):
        """Handle joystick button press"""
        if len(msg.buttons) <= self.button_idx:
            return
        
        # Detect rising edge (button press)
        current_state = msg.buttons[self.button_idx]
        
        if current_state == 1 and self.last_button_state == 0:
            self.save_waypoint()
        
        self.last_button_state = current_state
    
    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle in degrees"""
        # Extract quaternion components
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # Calculate yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees
        return math.degrees(yaw)
    
    def save_waypoint(self):
        """Save current robot position as rack waypoint"""
        if self.current_pose is None:
            self.get_logger().warn("⚠️  No pose available yet! Wait for localization...")
            return
        
        self.rack_count += 1
        
        # Calculate yaw angle
        yaw_deg = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Create waypoint data
        waypoint = {
            'id': self.rack_count,
            'name': f'RACK_{self.rack_count}',
            'timestamp': datetime.now().isoformat(),
            'position': {
                'x': float(self.current_pose.position.x),
                'y': float(self.current_pose.position.y),
                'z': float(self.current_pose.position.z)
            },
            'orientation': {
                'x': float(self.current_pose.orientation.x),
                'y': float(self.current_pose.orientation.y),
                'z': float(self.current_pose.orientation.z),
                'w': float(self.current_pose.orientation.w)
            },
            'yaw_degrees': float(yaw_deg)
        }
        
        self.waypoints.append(waypoint)
        
        # Save to file
        output_data = {
            'total_racks': self.rack_count,
            'created': datetime.now().isoformat(),
            'arena': 'warehouse',
            'waypoints': self.waypoints
        }
        
        try:
            with open(self.output_file, 'w') as f:
                json.dump(output_data, f, indent=2)
            
            # Print confirmation
            self.get_logger().info("╔════════════════════════════════════════╗")
            self.get_logger().info(f"║  ✅ RACK {self.rack_count} SAVED!                   ║")
            self.get_logger().info("╚════════════════════════════════════════╝")
            self.get_logger().info(f"📍 Position: ({waypoint['position']['x']:.3f}, "
                                  f"{waypoint['position']['y']:.3f})")
            self.get_logger().info(f"🧭 Orientation: {yaw_deg:.1f}°")
            self.get_logger().info(f"📊 Total waypoints: {self.rack_count}")
            self.get_logger().info("")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save waypoint: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("")
        node.get_logger().info("╔════════════════════════════════════════╗")
        node.get_logger().info(f"║  💾 SESSION COMPLETE                  ║")
        node.get_logger().info("╚════════════════════════════════════════╝")
        node.get_logger().info(f"📊 Total waypoints saved: {node.rack_count}")
        node.get_logger().info(f"💾 File: {node.output_file}")
        node.get_logger().info("")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
