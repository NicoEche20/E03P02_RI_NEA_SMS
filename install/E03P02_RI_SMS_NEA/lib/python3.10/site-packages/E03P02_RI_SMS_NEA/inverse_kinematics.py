#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        
        # Subscribe to desired poses
        self.pose_sub = self.create_subscription(
            Pose, 'desired_pose', self.pose_callback, 10
        )
        
        # Publish joint commands
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        
        # Robot parameters
        self.arm1_length = 0.5
        self.arm2_length = 0.4
        self.base_height = 0.2
        
        # Joint limits [min, max]
        self.joint_limits = {
            'joint1': [-math.pi/2, math.pi/2],    # ±90° base rotation
            'joint2': [-math.pi/2, math.pi/2],    # ±90° elbow
            'joint3': [0.0, 0.3]                  # Prismatic limits
        }
        
        self.get_logger().info('Inverse Kinematics node started with joint limits')

    def pose_callback(self, msg):
        """Calculate joint angles for desired end-effector position with limits"""
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        self.get_logger().info(f'Calculating IK for position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Calculate joint values using geometric approach with limits
        try:
            # Joint 1: Base rotation (limited to ±90°)
            theta1 = math.atan2(y, x)
            theta1 = self.apply_joint_limit(theta1, 'joint1')
            
            # Adjust for base height
            z_eff = z - self.base_height
            
            # Distance from base to target in XY plane
            r = math.sqrt(x**2 + y**2)
            
            # Joint 2: Elbow angle with limits
            distance = math.sqrt(r**2 + z_eff**2)
            
            # Law of cosines to find theta2
            cos_theta2 = (self.arm1_length**2 + distance**2 - self.arm2_length**2) / (2 * self.arm1_length * distance)
            cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # Clamp to valid range
            
            theta2 = math.acos(cos_theta2) + math.atan2(z_eff, r)
            theta2 = self.apply_joint_limit(theta2, 'joint2')
            
            # Joint 3: Prismatic extension with limits
            S3 = max(0.0, min(0.3, distance - self.arm1_length - 0.1))
            S3 = self.apply_joint_limit(S3, 'joint3')
            
            # Publish joint commands
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ['joint1', 'joint2', 'joint3']
            joint_msg.position = [theta1, theta2, S3]
            
            self.joint_pub.publish(joint_msg)
            
            self.get_logger().info(f'IK Solution: θ1={theta1:.3f} ({math.degrees(theta1):.1f}°), '
                                  f'θ2={theta2:.3f} ({math.degrees(theta2):.1f}°), S3={S3:.3f}')
            
        except Exception as e:
            self.get_logger().error(f'IK calculation failed: {e}')

    def apply_joint_limit(self, value, joint_name):
        """Apply joint limits to a joint value"""
        min_limit, max_limit = self.joint_limits[joint_name]
        
        if value < min_limit:
            self.get_logger().warn(f'Joint {joint_name} limited from {value:.3f} to {min_limit:.3f}')
            return min_limit
        elif value > max_limit:
            self.get_logger().warn(f'Joint {joint_name} limited from {value:.3f} to {max_limit:.3f}')
            return max_limit
        else:
            return value

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()