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
        
        self.get_logger().info('Forward Kinematics node started')
    
    def dh_matrix(self, a, alpha, d, theta):
        """Create DH transformation matrix"""
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_alpha = math.cos(alpha)
        sin_alpha = math.sin(alpha)
        
        return np.array([
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])
    
    def joint_callback(self, msg):
        """Calculate end-effector pose from joint states using DH parameters"""
        try:
            # Extract joint values
            theta1 = msg.position[0]  # Joint 1 rotation
            theta2 = msg.position[1]  # Joint 2 rotation  
            S3 = msg.position[2]      # Joint 3 prismatic
            
            # DH parameters for RRP robot
            # [a, alpha, d, theta]
            dh_params = [
                [0.5, 0, 0.2, theta1],      # Joint 1
                [0.4, 0, 0, theta2],        # Joint 2
                [0, math.pi/2, 0.15 + S3, 0] # Joint 3 (prismatic)
            ]
            
            # Calculate transformation matrices
            T01 = self.dh_matrix(*dh_params[0])
            T12 = self.dh_matrix(*dh_params[1])
            T23 = self.dh_matrix(*dh_params[2])
            
            # Total transformation from base to end-effector
            T03 = T01 @ T12 @ T23
            
            # Extract position
            x = T03[0, 3]
            y = T03[1, 3]
            z = T03[2, 3]
            
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