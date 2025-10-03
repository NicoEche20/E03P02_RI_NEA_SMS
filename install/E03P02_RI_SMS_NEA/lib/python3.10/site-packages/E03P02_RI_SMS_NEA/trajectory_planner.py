#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
import time

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # Subscribe to CAD waypoints
        self.waypoints_sub = self.create_subscription(
            PoseArray, 'cad_waypoints', self.waypoints_callback, 10
        )
        
        # Publish desired poses for IK
        self.desired_pose_pub = self.create_publisher(Pose, 'desired_pose', 10)
        
        self.current_waypoints = []
        self.trajectory_points = []
        self.current_point_index = 0
        self.last_publish_time = 0
        
        # Slower trajectory parameters for smoother motion
        self.point_duration = 0.1  # 10 Hz - slower for smoother motion
        self.interpolation_points = 30  # More points for smoother curves
        
        # Timer for smooth trajectory execution
        self.timer = self.create_timer(self.point_duration, self.execute_trajectory)
        
        self.get_logger().info('Trajectory Planner node started - Smooth slow motion enabled')

    def waypoints_callback(self, msg):
        """Receive CAD waypoints and create smooth trajectory"""
        if not msg.poses:
            return
            
        self.current_waypoints = msg.poses
        self.get_logger().info(f'Received {len(self.current_waypoints)} waypoints')
        
        # Create smooth trajectory
        self.trajectory_points = self.create_smooth_trajectory(self.current_waypoints)
        self.current_point_index = 0
        self.last_publish_time = time.time()
        
        self.get_logger().info(f'Created smooth trajectory with {len(self.trajectory_points)} points')

    def create_smooth_trajectory(self, waypoints):
        """Create smooth trajectory using cubic spline interpolation"""
        if len(waypoints) < 2:
            return waypoints
            
        trajectory_points = []
        
        # Add interpolation between all waypoints
        for i in range(len(waypoints)):
            start_pose = waypoints[i]
            end_pose = waypoints[(i + 1) % len(waypoints)]  # Loop back to start
            
            # Add smooth interpolation between waypoints
            for j in range(self.interpolation_points):
                t = j / float(self.interpolation_points)
                
                # Cubic spline interpolation for very smooth motion
                t2 = t * t
                t3 = t2 * t
                
                # Smooth easing function
                ease_t = t3 * (t * (6*t - 15) + 10)  # Smoother step function
                
                # Interpolate position with easing
                x = start_pose.position.x + (end_pose.position.x - start_pose.position.x) * ease_t
                y = start_pose.position.y + (end_pose.position.y - start_pose.position.y) * ease_t
                z = start_pose.position.z + (end_pose.position.z - start_pose.position.z) * ease_t
                
                # Create interpolated pose
                interpolated_pose = Pose()
                interpolated_pose.position.x = float(x)
                interpolated_pose.position.y = float(y)
                interpolated_pose.position.z = float(z)
                interpolated_pose.orientation = start_pose.orientation
                
                trajectory_points.append(interpolated_pose)
        
        return trajectory_points

    def execute_trajectory(self):
        """Execute smooth continuous trajectory"""
        if not self.trajectory_points:
            return
            
        current_time = time.time()
        
        # Only publish if enough time has passed (rate limiting)
        if current_time - self.last_publish_time < self.point_duration - 0.01:
            return
            
        # Get current point and publish it
        pose = self.trajectory_points[self.current_point_index]
        self.desired_pose_pub.publish(pose)
        
        # Move to next point
        self.current_point_index = (self.current_point_index + 1) % len(self.trajectory_points)
        self.last_publish_time = current_time
        
        # Log progress occasionally
        if self.current_point_index % 50 == 0:
            self.get_logger().info(f'Executing point {self.current_point_index}/{len(self.trajectory_points)}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()