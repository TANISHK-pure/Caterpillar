#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrafficCopNode(Node):
    def __init__(self):
        super().__init__('traffic_cop_node')
        
        # In a real scenario, we'd subscribe to /truck1/odom and /truck2/odom.
        # For this headless test, we will simulate the incoming telemetry.
        self.timer = self.create_timer(2.0, self.milp_traffic_loop)
        
        # Publishers to physically slow down the trucks in Gazebo
        self.truck1_speed_pub = self.create_publisher(Twist, '/truck1/cmd_vel', 10)
        self.truck2_speed_pub = self.create_publisher(Twist, '/truck2/cmd_vel', 10)
        
        self.get_logger().info("🚦 Live Traffic Controller [ONLINE]: Monitoring Haul Roads...")
        
        # Simulated live telemetry: [Distance to pit (m), Current Speed (m/s)]
        self.truck1_state = {'dist': 500.0, 'speed': 10.0}
        self.truck2_state = {'dist': 520.0, 'speed': 12.0} # Driving faster, will cause a crash
        
        self.safe_headway_time = 15.0 # Seconds required between arrivals

    def milp_traffic_loop(self):
        # 1. Calculate ETAs
        eta1 = self.truck1_state['dist'] / self.truck1_state['speed']
        eta2 = self.truck2_state['dist'] / self.truck2_state['speed']
        
        self.get_logger().info(f"Tracking... T1 ETA: {eta1:.1f}s | T2 ETA: {eta2:.1f}s")
        
        # 2. Conflict Detection (MILP Logic)
        time_difference = abs(eta1 - eta2)
        
        if time_difference < self.safe_headway_time:
            self.get_logger().warn(f"⚠️ DEADLOCK RISK DETECTED: Trucks arriving within {time_difference:.1f}s of each other!")
            
            # 3. Conflict Resolution: Force Truck 2 to slow down
            target_eta2 = eta1 + self.safe_headway_time
            new_safe_speed = self.truck2_state['dist'] / target_eta2
            
            self.get_logger().info(f"   -> RESOLUTION: Commanding Truck 2 to slow down to {new_safe_speed:.2f} m/s")
            
            # 4. Actually send the command to the truck's motors
            speed_cmd = Twist()
            speed_cmd.linear.x = new_safe_speed
            self.truck2_speed_pub.publish(speed_cmd)
            
            # Update our simulation to reflect the new speed
            self.truck2_state['speed'] = new_safe_speed
        else:
            self.get_logger().info("✅ Haul road clear. Maintain current speeds.")
            
        # Simulate trucks moving forward for the next loop
        self.truck1_state['dist'] -= self.truck1_state['speed'] * 2.0
        self.truck2_state['dist'] -= self.truck2_state['speed'] * 2.0

def main(args=None):
    rclpy.init(args=args)
    node = TrafficCopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
