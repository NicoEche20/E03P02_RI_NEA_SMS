#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import math
import time

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        # Subscribe to end-effector pose
        self.pose_sub = self.create_subscription(
            Pose, 
            'end_effector_pose', 
            self.pose_callback, 
            10
        )
        
        # Publisher for path
        self.path_pub = self.create_publisher(Path, 'end_effector_path', 10)
        
        # Store path history
        self.path_history = Path()
        self.path_history.header.frame_id = "base_link"
        self.max_path_length = 500  # Reduced to prevent immediate drawing
        
        # Movement tracking
        self.last_pose = None
        self.movement_threshold = 0.005  # Only add point if moved > 5mm
        self.last_add_time = 0
        self.min_time_between_points = 0.2  # Only add point every 0.2 seconds
        
        # Timer to publish path periodically
        self.timer = self.create_timer(0.5, self.publish_path)  # Slower publishing
        
        self.get_logger().info('Path Publisher node started - Only showing meaningful movement')

    def pose_callback(self, msg):
        """Add new pose to path history only if meaningful movement occurred"""
        current_time = time.time()
        
        # Check if enough time has passed since last point
        if current_time - self.last_add_time < self.min_time_between_points:
            return
            
        # Check if this is meaningful movement from last pose
        if self.last_pose is not None:
            distance = math.sqrt(
                (msg.position.x - self.last_pose.position.x) ** 2 +
                (msg.position.y - self.last_pose.position.y) ** 2 +
                (msg.position.z - self.last_pose.position.z) ** 2
            )
            
            # Only add point if movement is significant
            if distance < self.movement_threshold:
                return
        
        # Add the new pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = msg
        
        self.path_history.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.path_history.poses) > self.max_path_length:
            self.path_history.poses.pop(0)
        
        # Update header and tracking
        self.path_history.header.stamp = pose_stamped.header.stamp
        self.last_pose = msg
        self.last_add_time = current_time

    def publish_path(self):
        """Publish the current path"""
        if self.path_history.poses:
            self.path_pub.publish(self.path_history)
            
            # Log occasionally
            if len(self.path_history.poses) % 10 == 0:
                self.get_logger().info(f'Path has {len(self.path_history.poses)} points')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()