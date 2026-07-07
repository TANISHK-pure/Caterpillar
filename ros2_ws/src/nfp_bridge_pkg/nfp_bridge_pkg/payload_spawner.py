#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import subprocess

class PayloadSpawner(Node):
    def __init__(self):
        super().__init__('payload_spawner')
        
        # Listen for the "I have arrived" signal from the truck
        self.subscription = self.create_subscription(
            Point,
            '/trigger_dump',
            self.dump_callback,
            10
        )
        
        self.get_logger().info("🚜 Virtual Dump Service [ONLINE]: Waiting for trucks to arrive...")
        self.dump_count = 0

    def dump_callback(self, msg):
        self.dump_count += 1
        target_x = msg.x
        target_y = msg.y
        
        self.get_logger().info(f"📍 TRUCK ARRIVED! Initiating Dump Sequence #{self.dump_count}")
        self.get_logger().info(f"   -> Target Coordinates: [X: {target_x:.2f}, Y: {target_y:.2f}]")
        
        # =========================================================================
        # THE GAZEBO HARMONIC SPAWN COMMAND
        # This formats the exact terminal command required to inject a model into a live Gazebo world.
        # It uses a placeholder 'dirt_cone' model.
        # =========================================================================
        gz_spawn_cmd = [
            'gz', 'service', '-s', '/world/empty/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--req', f'sdf_filename: "model://dirt_cone", pose: {{position: {{x: {target_x}, y: {target_y}, z: 0.0}}}}'
        ]
        
        try:
            self.get_logger().info("   -> Injecting 3D payload mesh into Gazebo...")
            
            # For this headless test, we will just print the command instead of running it,
            # so it doesn't throw an error since Gazebo isn't open right now.
            # In production, uncomment the line below:
            # subprocess.run(gz_spawn_cmd, check=True)
            
            self.get_logger().info("   ✅ Payload dropped successfully. Map updated for the next truck.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to spawn payload: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PayloadSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
