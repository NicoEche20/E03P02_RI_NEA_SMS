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
        
        # Store path history - SIN LÍMITE para dibujo completo
        self.path_history = Path()
        self.path_history.header.frame_id = "base_link"
        self.max_path_length = 5000  # Más grande para el dibujo completo
        
        # Movement tracking
        self.last_pose = None
        self.movement_threshold = 0.003  # 3mm - más sensible
        self.last_add_time = 0
        self.min_time_between_points = 0.05  # Más rápido: cada 50ms
        
        # Clear flag - para resetear el path cuando sea necesario
        self.should_clear = False
        
        # Timer to publish path - MÁS FRECUENTE
        self.timer = self.create_timer(0.1, self.publish_path)  # 10 Hz
        
        self.get_logger().info('Path Publisher started - Persistent path for complete drawing')
    
    def pose_callback(self, msg):
        """Add new pose to path history with smart filtering"""
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
            
            # Detect large jumps (approach movements) - don't draw these
            if distance > 0.1:  # 10cm jump = approach movement
                self.get_logger().info(f'Large jump detected ({distance:.3f}m), skipping path point')
                self.last_pose = msg
                self.last_add_time = current_time
                return
        
        # Add the new pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = msg
        
        self.path_history.poses.append(pose_stamped)
        
        # Limit path length (pero con límite MUY grande)
        if len(self.path_history.poses) > self.max_path_length:
            # Remove oldest points, not clear everything
            self.path_history.poses = self.path_history.poses[-self.max_path_length:]
            self.get_logger().info('Path limit reached, removing oldest points')
        
        # Update header and tracking
        self.path_history.header.stamp = pose_stamped.header.stamp
        self.last_pose = msg
        self.last_add_time = current_time
    
    def publish_path(self):
        """Publish the current path"""
        if self.path_history.poses:
            self.path_pub.publish(self.path_history)
            
            # Log occasionally
            if len(self.path_history.poses) % 50 == 0:
                self.get_logger().info(
                    f'Path has {len(self.path_history.poses)} points',
                    throttle_duration_sec=2.0
                )
    
    def clear_path(self):
        """Clear the path history (call this to reset)"""
        self.path_history.poses.clear()
        self.last_pose = None
        self.get_logger().info('Path cleared')

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

if __name__ == 'main':
    main()