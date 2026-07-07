#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose

class NFPBridgeNode(Node):
    def __init__(self):
        super().__init__('nfp_to_nav2_bridge')
        
        # 1. Listen to your MATLAB Master Node
        self.subscription = self.create_subscription(
            Point,
            '/gazebo_truck_targets',
            self.matlab_callback,
            10
        )
        
        # 2. Setup the Action Client to talk to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Headless Bridge [ONLINE]: Waiting for MATLAB coordinates...")

    def matlab_callback(self, msg):
        target_x = msg.x
        target_y = msg.y
        self.get_logger().info(f"Received NFP Target -> X: {target_x}, Y: {target_y}")
        self.send_nav_goal(target_x, target_y)

    def send_nav_goal(self, x, y):
        # Wait for the Nav2 server to actually be running
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server not available! (Is Nav2 running?)")
            return

        # Package the coordinate into a Nav2 Pose message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Default quaternion facing forward
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending formatted target to Nav2 Hybrid A*...')
        self.nav_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NFPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
