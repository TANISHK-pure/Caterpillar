#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan  # NEW: Import LiDAR messages

class NFPFleetPilot(Node):
    def __init__(self):
        super().__init__('fleet_node')
        
        self.pub_797 = self.create_publisher(Twist, '/model/truck_797/cmd_vel', 10)
        self.pub_789 = self.create_publisher(Twist, '/model/truck_789/cmd_vel', 10)
        self.telemetry_pub = self.create_publisher(String, '/fleet_telemetry', 10)
        
        # Subscribe to the MILP override and the new 3D LiDAR!
        self.override_sub = self.create_subscription(String, '/speed_override', self.override_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/model/truck_797/lidar', self.lidar_callback, 10)
        
        self.v_max = 0.5       
        self.k_rep = 2.0       
        self.d0 = 6.0          
        
        # NEW: Store the actual physical distance from the lasers
        self.physical_lidar_distance = 20.0 
        
        self.trucks = {
            'Cat_797F': {'dist': 10.0, 'spd': 0.0, 'milp_lock': 0},
            'Cat_789C': {'dist': 8.0,  'spd': 0.0, 'milp_lock': 0}
        }
        
        self.timer = self.create_timer(0.1, self.nfp_control_loop)
        self.get_logger().info('LiDAR-Enabled NFP Architecture Online...')

    def override_callback(self, msg):
        command = msg.data.split(',')
        target_truck = command[0]
        if target_truck in self.trucks:
            self.trucks[target_truck]['milp_lock'] = 5

    def lidar_callback(self, msg):
        """ Read the lasers and find the closest physical object """
        valid_ranges = [r for r in msg.ranges if r > 0.2 and r != float('inf')]
        if valid_ranges:
            self.physical_lidar_distance = min(valid_ranges)
        else:
            self.physical_lidar_distance = 20.0

    def calculate_nfp_velocity(self, lock_timer, use_lidar=False):
        if lock_timer > 0:
            return 0.0
            
        v_cmd = self.v_max 
        
        # If this is the 797, use the actual LiDAR distance to calculate the repulsive force!
        if use_lidar:
            if self.physical_lidar_distance < self.d0:
                self.get_logger().info(f'LiDAR DETECTED TRUCK AT: {self.physical_lidar_distance:.2f}m! Braking...')
                repulsive_push = self.k_rep * ((1.0 / self.physical_lidar_distance) - (1.0 / self.d0))**2
                v_cmd -= repulsive_push 
        
        if v_cmd < 0.0:
            v_cmd = 0.0
            
        return v_cmd

    def nfp_control_loop(self):
        for t in self.trucks.values():
            if t['milp_lock'] > 0:
                t['milp_lock'] -= 1

        # Calculate speed. 797 uses true LiDAR. 789 uses basic cruise control.
        spd_797 = self.calculate_nfp_velocity(self.trucks['Cat_797F']['milp_lock'], use_lidar=True)
        spd_789 = self.calculate_nfp_velocity(self.trucks['Cat_789C']['milp_lock'], use_lidar=False)

        # GPS/Odometry simulation for the Traffic Cop
        if self.trucks['Cat_797F']['dist'] > -5.0:
            self.trucks['Cat_797F']['dist'] -= spd_797 * 0.1
        if self.trucks['Cat_789C']['dist'] > -5.0:
            self.trucks['Cat_789C']['dist'] -= spd_789 * 0.1

        msg_797 = String()
        msg_797.data = f"Cat_797F,{self.trucks['Cat_797F']['dist']},{spd_797}"
        self.telemetry_pub.publish(msg_797)
        
        msg_789 = String()
        msg_789.data = f"Cat_789C,{self.trucks['Cat_789C']['dist']},{spd_789}"
        self.telemetry_pub.publish(msg_789)

        t_797 = Twist()
        t_797.linear.x = spd_797
        self.pub_797.publish(t_797)

        t_789 = Twist()
        t_789.linear.x = spd_789
        self.pub_789.publish(t_789)

def main(args=None):
    rclpy.init(args=args)
    node = NFPFleetPilot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
