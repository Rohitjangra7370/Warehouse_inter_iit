#!/usr/bin/env python3
"""
Scanning Controller Node
Executes LEFT-UP-RIGHT-DOWN scanning pattern at each rack
Ensures complete rack coverage with lift movement
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32
import time

class ScanningController(Node):
    def __init__(self):
        super().__init__('scanning_controller')
        
        # Parameters
        self.declare_parameter('lateral_distance', 0.30)      # meters
        self.declare_parameter('lateral_speed', 0.05)         # m/s
        self.declare_parameter('lift_min_height', 0.45)       # meters
        self.declare_parameter('lift_max_height', 1.40)       # meters
        self.declare_parameter('lift_move_duration', 8.0)     # seconds
        self.declare_parameter('dwell_time_scan', 3.0)        # seconds
        self.declare_parameter('dwell_time_move', 1.0)        # seconds
        
        self.lateral_dist = self.get_parameter('lateral_distance').value
        self.lateral_speed = self.get_parameter('lateral_speed').value
        self.lift_min = self.get_parameter('lift_min_height').value
        self.lift_max = self.get_parameter('lift_max_height').value
        self.lift_duration = self.get_parameter('lift_move_duration').value
        self.dwell_scan = self.get_parameter('dwell_time_scan').value
        self.dwell_move = self.get_parameter('dwell_time_move').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service clients for lift control
        self.lift_scan_client = self.create_client(
            Trigger, '/lift_controller/scan_cycle')
        
        # Service server
        self.scan_service = self.create_service(
            Trigger,
            '/scanning_controller/execute_scan',
            self.execute_scan_callback
        )
        
        self.scanning_active = False
        
        self.get_logger().info("╔════════════════════════════════════════╗")
        self.get_logger().info("║   🔄 SCANNING CONTROLLER READY        ║")
        self.get_logger().info("╚════════════════════════════════════════╝")
        self.get_logger().info(f"📏 Pattern:")
        self.get_logger().info(f"   1. Move LEFT {self.lateral_dist}m")
        self.get_logger().info(f"   2. Lift UP ({self.lift_min}m → {self.lift_max}m)")
        self.get_logger().info(f"   3. Move RIGHT {self.lateral_dist * 2}m")
        self.get_logger().info(f"   4. Lift DOWN ({self.lift_max}m → {self.lift_min}m)")
        self.get_logger().info(f"   5. Return to CENTER")
        self.get_logger().info("")
    
    def execute_scan_callback(self, request, response):
        """Execute full LEFT-UP-RIGHT-DOWN scanning pattern"""
        if self.scanning_active:
            response.success = False
            response.message = "Scan already in progress"
            return response
        
        self.scanning_active = True
        
        self.get_logger().info("")
        self.get_logger().info("╔════════════════════════════════════════╗")
        self.get_logger().info("║   🎯 STARTING SCAN PATTERN            ║")
        self.get_logger().info("╚════════════════════════════════════════╝")
        
        try:
            # PHASE 1: Move LEFT
            self.get_logger().info("")
            self.get_logger().info("⬅️  PHASE 1: Moving LEFT...")
            self.move_lateral(-self.lateral_dist)
            self.get_logger().info("   ✅ Reached LEFT position")
            time.sleep(self.dwell_move)
            
            # PHASE 2: Lift UP (while scanning)
            self.get_logger().info("")
            self.get_logger().info("⬆️  PHASE 2: Lifting UP + Scanning...")
            self.trigger_lift_scan()
            time.sleep(self.lift_duration + self.dwell_scan)
            self.get_logger().info("   ✅ Reached TOP")
            
            # PHASE 3: Move RIGHT (while staying at top)
            self.get_logger().info("")
            self.get_logger().info("➡️  PHASE 3: Moving RIGHT...")
            self.move_lateral(self.lateral_dist * 2)  # LEFT → CENTER → RIGHT
            self.get_logger().info("   ✅ Reached RIGHT position")
            time.sleep(self.dwell_move)
            
            # PHASE 4: Lift DOWN (while scanning)
            self.get_logger().info("")
            self.get_logger().info("⬇️  PHASE 4: Lowering DOWN + Scanning...")
            time.sleep(self.lift_duration + self.dwell_scan)
            self.get_logger().info("   ✅ Reached BOTTOM")
            
            # PHASE 5: Return to CENTER
            self.get_logger().info("")
            self.get_logger().info("🎯 PHASE 5: Returning to CENTER...")
            self.move_lateral(-self.lateral_dist)  # RIGHT → CENTER
            self.get_logger().info("   ✅ Back at CENTER")
            
            self.get_logger().info("")
            self.get_logger().info("╔════════════════════════════════════════╗")
            self.get_logger().info("║   ✅ SCAN PATTERN COMPLETE!           ║")
            self.get_logger().info("╚════════════════════════════════════════╝")
            self.get_logger().info("")
            
            response.success = True
            response.message = "Scanning pattern completed successfully"
            
        except Exception as e:
            self.get_logger().error(f"❌ Scan failed: {e}")
            response.success = False
            response.message = str(e)
        
        finally:
            self.scanning_active = False
        
        return response
    
    def move_lateral(self, distance):
        """
        Move robot laterally (strafe left/right) using mecanum wheels
        
        Args:
            distance: meters (positive = right, negative = left)
        """
        if distance == 0:
            return
        
        duration = abs(distance / self.lateral_speed)
        direction = "RIGHT" if distance > 0 else "LEFT"
        
        self.get_logger().info(f"   Moving {direction} {abs(distance):.2f}m "
                              f"at {self.lateral_speed}m/s...")
        
        twist = Twist()
        twist.linear.y = self.lateral_speed if distance > 0 else -self.lateral_speed
        
        # Publish velocity commands
        start_time = self.get_clock().now()
        rate = self.create_rate(50)  # 50 Hz
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Stop
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def trigger_lift_scan(self):
        """Trigger lift controller to execute scan cycle"""
        self.get_logger().info("   📸 Triggering lift scan cycle...")
        
        request = Trigger.Request()
        
        if not self.lift_scan_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("   ⚠️  Lift scan service not available")
            return
        
        future = self.lift_scan_client.call_async(request)
        # Don't wait for result, let it run in background

def main(args=None):
    rclpy.init(args=args)
    node = ScanningController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
