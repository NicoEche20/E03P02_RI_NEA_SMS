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
        
        
        self.arm1_length = 0.5      # First revolute arm
        self.arm2_length = 0.4      # Second revolute arm  
        self.base_height = 0.2      # Base height from URDF
        self.min_prismatic = 0.0    # Minimum prismatic extension
        self.max_prismatic = 0.3    # Maximum prismatic extension
        
        # Joint limits [min, max]
        self.joint_limits = {
            'joint1': [-math.pi/2, math.pi/2],    # ±90° base rotation
            'joint2': [-math.pi/2, math.pi/2],    # ±90° elbow
            'joint3': [self.min_prismatic, self.max_prismatic]  # Prismatic limits
        }
        
        self.get_logger().info('Inverse Kinematics node started with joint limits')

    def pose_callback(self, msg):
        """Calculate joint angles for desired end-effector position with limits"""
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        # Skip zero poses (initialization)
        if abs(x) < 0.001 and abs(y) < 0.001 and abs(z) < 0.001:
            return

        self.get_logger().info(f'Calculating IK for position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        try:
            # Calculate joint values using geometric approach
            # For RRP arm: base rotation -> elbow rotation -> prismatic extension
            
            # Joint 1: Base rotation (atan2 handles all quadrants)
            theta1 = math.atan2(y, x)
            theta1 = self.apply_joint_limit(theta1, 'joint1')
            
            # Distance from base to target in XY plane (projected)
            r_xy = math.sqrt(x**2 + y**2)
            
            # Adjust for base height to get effective Z
            z_eff = z - self.base_height
            
            
            # The two revolute joints determine the direction, prismatic extends
            
            # Joint 2: This is the elevation angle of the entire arm assembly
            # In an RRP robot, theta2 directly controls the pitch angle
            theta2 = math.atan2(z_eff, r_xy)
            theta2 = self.apply_joint_limit(theta2, 'joint2')
            
            # Calculate required total length to reach the point
            required_length = math.sqrt(r_xy**2 + z_eff**2)
            
            # Joint 3: Prismatic extension
            # Total length = arm1 + arm2 + prismatic
            S3 = required_length - (self.arm1_length + self.arm2_length)
            S3 = self.apply_joint_limit(S3, 'joint3')
            
            # Check reachability
            min_reach = self.arm1_length + self.arm2_length + self.min_prismatic
            max_reach = self.arm1_length + self.arm2_length + self.max_prismatic
            
            if S3 < self.min_prismatic or S3 > self.max_prismatic:
                self.get_logger().warn(f'Target unreachable: S3={S3:.3f}, '
                                      f'range=[{self.min_prismatic:.3f}, {self.max_prismatic:.3f}]')
                return
            
            # Verify the solution with forward kinematics
            fk_x, fk_y, fk_z = self.forward_kinematics(theta1, theta2, S3)
            error = math.sqrt((x - fk_x)**2 + (y - fk_y)**2 + (z - fk_z)**2)
            
            if error > 0.01:  # 1cm tolerance
                self.get_logger().warning(f'IK-FK mismatch! Target: ({x:.3f}, {y:.3f}, {z:.3f})')
                self.get_logger().warning(f'FK Result: ({fk_x:.3f}, {fk_y:.3f}, {fk_z:.3f})')
                self.get_logger().warning(f'Error: {error:.3f}m')
            
            # Publish joint commands
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ['joint1', 'joint2', 'joint3']
            joint_msg.position = [theta1, theta2, S3]
            joint_msg.velocity = [0.1, 0.1, 0.05]  
            joint_msg.effort = [0.0, 0.0, 0.0]
            
            self.joint_pub.publish(joint_msg)
            
            self.get_logger().info(f'IK Solution: θ1={theta1:.3f}rad ({math.degrees(theta1):.1f}°), '
                                  f'θ2={theta2:.3f}rad ({math.degrees(theta2):.1f}°), S3={S3:.3f}m')
            self.get_logger().info(f'FK Verification: x={fk_x:.3f}, y={fk_y:.3f}, z={fk_z:.3f}, error={error:.3f}m')
            
        except Exception as e:
            self.get_logger().error(f'IK calculation failed: {e}')

    def forward_kinematics(self, theta1, theta2, S3):
        """Calculate end-effector position from joint angles"""
        total_length = self.arm1_length + self.arm2_length + S3
        
        x = total_length * math.cos(theta1) * math.cos(theta2)
        y = total_length * math.cos(theta2) * math.sin(theta1)  
        z = self.base_height + total_length * math.sin(theta2)
        
        return x, y, z

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