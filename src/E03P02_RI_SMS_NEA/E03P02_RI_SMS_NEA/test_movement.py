#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class TestMovement(Node):
    def __init__(self):
        super().__init__('test_movement')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        
        # Timer to move robot
        self.timer = self.create_timer(1.0, self.move_robot)
        self.angle = 0.0
        
        self.get_logger().info('Test Movement node started - Robot should move now!')
    
    def move_robot(self):
        """Move robot in a simple pattern"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3']
        
        # Simple oscillating movement
        self.angle += 0.2
        joint_msg.position = [
            math.sin(self.angle) * 1.0,      # joint1 - base rotation
            math.cos(self.angle) * 0.5,      # joint2 - elbow
            (math.sin(self.angle) + 1) * 0.1 # joint3 - prismatic
        ]
        
        joint_msg.velocity = [0.1, 0.1, 0.05]
        joint_msg.effort = [0.0, 0.0, 0.0]
        
        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f'Moving robot: {joint_msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = TestMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()