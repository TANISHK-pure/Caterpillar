#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MILPIntersectionManager(Node):
    def __init__(self):
        super().__init__('traffic_cop')
        self.truck_data = {}
        
        # Define the Critical Zone (e.g., a 5-meter wide intersection box)
        self.zone_radius = 2.5 
        
        self.telemetry_sub = self.create_subscription(String, '/fleet_telemetry', self.telemetry_callback, 10)
        self.override_pub = self.create_publisher(String, '/speed_override', 10)
        
        self.get_logger().info('Advanced MILP Solver Online. Monitoring multi-agent scheduling...')

    def telemetry_callback(self, msg):
        data = msg.data.split(',')
        name = data[0]
        dist = float(data[1])
        spd = float(data[2])
        
        self.truck_data[name] = {'dist': dist, 'spd': spd}
        self.solve_intersection()

    def solve_intersection(self):
        # Need at least two trucks to have a collision
        if len(self.truck_data) < 2:
            return

        schedule = []
        for name, data in self.truck_data.items():
            if data['spd'] > 0.05: # Only calculate for trucks that are actively driving
                # Calculate exactly when the front enters and rear exits the zone
                t_enter = (data['dist'] - self.zone_radius) / data['spd']
                t_exit = (data['dist'] + self.zone_radius) / data['spd']
                schedule.append({'name': name, 't_enter': t_enter, 't_exit': t_exit, 'dist': data['dist']})

        # Sort the queue by who naturally reaches the intersection first (y_ij = 1)
        schedule.sort(key=lambda x: x['t_enter'])

        # Evaluate the MILP overlap constraints
        for i in range(len(schedule) - 1):
            lead_truck = schedule[i]
            trailing_truck = schedule[i+1]

            # Constraint: Does the trailing truck enter BEFORE the lead truck completely exits?
            if trailing_truck['t_enter'] < lead_truck['t_exit']:
                
                # Only apply the brakes if they are within a 15-meter planning horizon
                if trailing_truck['dist'] < 5.0:
                    self.get_logger().warn(f"MILP CONFLICT: {trailing_truck['name']} yielding to {lead_truck['name']}")
                    
                    # Fire the hard stop command across the ROS 2 network
                    cmd_msg = String()
                    cmd_msg.data = f"{trailing_truck['name']},0.0"
                    self.override_pub.publish(cmd_msg)
                    
                    # Temporarily zero out its speed in the solver's memory so it doesn't spam the network
                    self.truck_data[trailing_truck['name']]['spd'] = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = MILPIntersectionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
