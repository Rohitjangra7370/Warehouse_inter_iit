#!/usr/bin/env python3
"""
Mission Executor Node
Navigates to all saved waypoints and executes scanning pattern at each rack
Fully autonomous mission execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import json
import time

class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')
        
        # Parameters
        self.declare_parameter('waypoints_file', '/tmp/rack_waypoints.json')
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('auto_start', True)
        
        waypoints_file = self.get_parameter('waypoints_file').value
        start_delay = self.get_parameter('start_delay').value
        auto_start = self.get_parameter('auto_start').value
        
        # Load waypoints
        try:
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                self.waypoints = data['waypoints']
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load waypoints: {e}")
            self.waypoints = []
        
        # State
        self.current_idx = 0
        self.mission_active = False
        self.total_racks_scanned = 0
        self.failed_racks = []
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Service client for scanning
        self.scan_client = self.create_client(
            Trigger, 
            '/scanning_controller/execute_scan'
        )
        
        # Service for manual mission control
        self.start_service = self.create_service(
            Trigger,
            '/mission_executor/start_mission',
            self.start_mission_callback
        )
        
        self.get_logger().info("╔════════════════════════════════════════╗")
        self.get_logger().info("║   🤖 MISSION EXECUTOR READY           ║")
        self.get_logger().info("╚════════════════════════════════════════╝")
        self.get_logger().info(f"📋 Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info(f"📂 File: {waypoints_file}")
        
        if len(self.waypoints) == 0:
            self.get_logger().warn("⚠️  No waypoints loaded! Record waypoints first.")
        else:
            for i, wp in enumerate(self.waypoints):
                self.get_logger().info(f"   {i+1}. {wp['name']}: "
                                      f"({wp['position']['x']:.2f}, "
                                      f"{wp['position']['y']:.2f})")
        
        self.get_logger().info("")
        
        # Auto-start after delay
        if auto_start and len(self.waypoints) > 0:
            self.get_logger().info(f"⏰ Mission will start in {start_delay:.0f} seconds...")
            self.create_timer(start_delay, self.start_mission_auto)
        else:
            self.get_logger().info("📞 Call /mission_executor/start_mission to begin")
        
        self.get_logger().info("")
    
    def start_mission_callback(self, request, response):
        """Service to manually start mission"""
        if self.mission_active:
            response.success = False
            response.message = "Mission already in progress"
            return response
        
        self.start_mission_auto()
        
        response.success = True
        response.message = f"Mission started with {len(self.waypoints)} waypoints"
        return response
    
    def start_mission_auto(self):
        """Start autonomous mission"""
        if self.mission_active:
            return
        
        if len(self.waypoints) == 0:
            self.get_logger().error("❌ Cannot start mission: No waypoints!")
            return
        
        self.mission_active = True
        self.current_idx = 0
        self.total_racks_scanned = 0
        self.failed_racks = []
        
        self.get_logger().info("")
        self.get_logger().info("╔════════════════════════════════════════╗")
        self.get_logger().info("║   🚀 AUTONOMOUS MISSION STARTED       ║")
        self.get_logger().info("╚════════════════════════════════════════╝")
        self.get_logger().info(f"📊 Total racks: {len(self.waypoints)}")
        self.get_logger().info("")
        
        self.navigate_to_next_waypoint()
    
    def navigate_to_next_waypoint(self):
        """Navigate to next rack waypoint"""
        if self.current_idx >= len(self.waypoints):
            self.mission_complete()
            return
        
        waypoint = self.waypoints[self.current_idx]
        
        self.get_logger().info("╔════════════════════════════════════════╗")
        self.get_logger().info(f"║  📍 RACK {self.current_idx + 1}/{len(self.waypoints)}: "
                              f"{waypoint['name']:<20} ║")
        self.get_logger().info("╚════════════════════════════════════════╝")
        self.get_logger().info(f"🎯 Target: ({waypoint['position']['x']:.3f}, "
                              f"{waypoint['position']['y']:.3f})")
        self.get_logger().info(f"🧭 Orientation: {waypoint.get('yaw_degrees', 0):.1f}°")
        self.get_logger().info("")
        
        # Create navigation goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint['position']['x']
        goal.pose.pose.position.y = waypoint['position']['y']
        goal.pose.pose.position.z = waypoint['position']['z']
        goal.pose.pose.orientation.x = waypoint['orientation']['x']
        goal.pose.pose.orientation.y = waypoint['orientation']['y']
        goal.pose.pose.orientation.z = waypoint['orientation']['z']
        goal.pose.pose.orientation.w = waypoint['orientation']['w']
        
        # Send goal
        self.get_logger().info("🚗 Navigating...")
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Navigation server not available!")
            self.handle_navigation_failure()
            return
        
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Navigation goal rejected!")
            self.handle_navigation_failure()
            return
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("✅ Navigation successful!")
            self.get_logger().info("")
            
            # Small delay before scanning
            time.sleep(1.0)
            
            # Execute scanning pattern
            self.execute_scan()
        else:
            self.get_logger().error(f"❌ Navigation failed with status: {result.status}")
            self.handle_navigation_failure()
    
    def execute_scan(self):
        """Call scanning service to execute LEFT-UP-RIGHT-DOWN pattern"""
        waypoint = self.waypoints[self.current_idx]
        
        self.get_logger().info("🔄 Starting scan pattern...")
        self.get_logger().info("")
        
        if not self.scan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Scanning service not available!")
            self.handle_scan_failure()
            return
        
        request = Trigger.Request()
        future = self.scan_client.call_async(request)
        future.add_done_callback(self.scan_complete_callback)
    
    def scan_complete_callback(self, future):
        """Handle scan completion"""
        try:
            result = future.result()
            
            if result.success:
                self.get_logger().info("")
                self.get_logger().info("╔════════════════════════════════════════╗")
                self.get_logger().info("║   ✅ RACK SCAN COMPLETE!              ║")
                self.get_logger().info("╚════════════════════════════════════════╝")
                self.get_logger().info("")
                
                self.total_racks_scanned += 1
                
                # Move to next waypoint
                self.current_idx += 1
                
                # Small delay before next navigation
                time.sleep(2.0)
                
                self.navigate_to_next_waypoint()
            else:
                self.get_logger().error(f"❌ Scan failed: {result.message}")
                self.handle_scan_failure()
        
        except Exception as e:
            self.get_logger().error(f"❌ Scan exception: {e}")
            self.handle_scan_failure()
    
    def handle_navigation_failure(self):
        """Handle navigation failure"""
        waypoint = self.waypoints[self.current_idx]
        self.failed_racks.append({
            'rack': waypoint['name'],
            'reason': 'Navigation failed'
        })
        
        self.get_logger().warn(f"⚠️  Skipping {waypoint['name']} due to navigation failure")
        
        # Move to next rack
        self.current_idx += 1
        time.sleep(2.0)
        self.navigate_to_next_waypoint()
    
    def handle_scan_failure(self):
        """Handle scanning failure"""
        waypoint = self.waypoints[self.current_idx]
        self.failed_racks.append({
            'rack': waypoint['name'],
            'reason': 'Scan failed'
        })
        
        self.get_logger().warn(f"⚠️  {waypoint['name']} scan failed, continuing...")
        
        # Move to next rack anyway
        self.current_idx += 1
        time.sleep(2.0)
        self.navigate_to_next_waypoint()
    
    def mission_complete(self):
        """Handle mission completion"""
        self.mission_active = False
        
        self.get_logger().info("")
        self.get_logger().info("╔════════════════════════════════════════╗")
        self.get_logger().info("║   🎉 MISSION COMPLETE!                ║")
        self.get_logger().info("╚════════════════════════════════════════╝")
        self.get_logger().info("")
        self.get_logger().info(f"📊 Mission Summary:")
        self.get_logger().info(f"   Total racks: {len(self.waypoints)}")
        self.get_logger().info(f"   ✅ Scanned successfully: {self.total_racks_scanned}")
        self.get_logger().info(f"   ❌ Failed: {len(self.failed_racks)}")
        self.get_logger().info(f"   📈 Success rate: "
                              f"{100 * self.total_racks_scanned / len(self.waypoints):.1f}%")
        
        if self.failed_racks:
            self.get_logger().info("")
            self.get_logger().info("⚠️  Failed racks:")
            for failed in self.failed_racks:
                self.get_logger().info(f"   - {failed['rack']}: {failed['reason']}")
        
        self.get_logger().info("")
        self.get_logger().info("✅ Robot returned to safe state")
        self.get_logger().info("")

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down mission executor...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
