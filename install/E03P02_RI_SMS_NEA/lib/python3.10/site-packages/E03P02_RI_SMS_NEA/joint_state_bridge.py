#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # Subscribe to your joint commands
        self.joint_sub = self.create_subscription(
            JointState, 
            'joint_commands', 
            self.joint_callback, 
            10
        )
        
        # Publish to standard joint_states topic
        self.joint_pub = self.create_publisher(
            JointState, 
            'joint_states', 
            10
        )
        
        # Current joint positions (home position)
        self.current_joint_positions = [0.0, 0.0, 0.0]
        self.has_valid_commands = False
        self.last_publish_time = 0
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info('Joint State Bridge node started')

    def joint_callback(self, msg):
        """Store latest joint commands - filter out zeros"""
        if len(msg.position) >= 3:
            # Check if this is a valid command (not all zeros and reasonable values)
            is_all_zeros = all(abs(pos) < 0.001 for pos in msg.position)
            has_reasonable_values = all(abs(pos) < 10.0 for pos in msg.position)  # Filter extreme values
            
            if not is_all_zeros and has_reasonable_values:
                self.current_joint_positions = [
                    float(msg.position[0]),
                    float(msg.position[1]), 
                    float(msg.position[2])
                ]
                self.has_valid_commands = True

    def publish_joint_states(self):
        """Publish joint states only when we have valid commands"""
        if not self.has_valid_commands:
            return
            
        current_time = time.time()
        # Rate limiting
        if current_time - self.last_publish_time < 0.09:  # ~11Hz max
            return
            
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3']
        joint_state.position = self.current_joint_positions
        joint_state.velocity = [0.1, 0.1, 0.05]  # Always include velocities
        joint_state.effort = [0.0, 0.0, 0.0]
        
        self.joint_pub.publish(joint_state)
        self.last_publish_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()