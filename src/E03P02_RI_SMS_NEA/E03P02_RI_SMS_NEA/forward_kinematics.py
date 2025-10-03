#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion

class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, 'joint_commands', self.joint_callback, 10
        )
        
        # Publish end-effector pose
        self.pose_pub = self.create_publisher(Pose, 'end_effector_pose', 10)
        
        # Robot parameters - MUST MATCH IK NODE
        self.arm1_length = 0.5      # First revolute arm
        self.arm2_length = 0.4      # Second revolute arm  
        self.base_height = 0.2      # Base height from URDF
        
        self.get_logger().info('Forward Kinematics node started')
    
    def joint_callback(self, msg):
        """Calculate end-effector pose from joint states using geometric approach"""
        try:
            # Extract joint values
            theta1 = msg.position[0]  # Joint 1 rotation
            theta2 = msg.position[1]  # Joint 2 rotation  
            S3 = msg.position[2]      # Joint 3 prismatic
            
           
            total_length = self.arm1_length + self.arm2_length + S3
            
            x = total_length * math.cos(theta1) * math.cos(theta2)
            y = total_length * math.cos(theta2) * math.sin(theta1)
            z = self.base_height + total_length * math.sin(theta2)
            
            # Create and publish pose
            pose = Pose()
            pose.position = Point(x=float(x), y=float(y), z=float(z))
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            self.pose_pub.publish(pose)
            
            self.get_logger().info(f'FK Result: x={x:.3f}, y={y:.3f}, z={z:.3f}', 
                                  throttle_duration_sec=1.0)
            
        except Exception as e:
            self.get_logger().error(f'FK calculation failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()