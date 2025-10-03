#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

class DXFParser(Node):
    def __init__(self):
        super().__init__('dxf_parser')
        
        # Publisher for waypoints
        self.waypoints_pub = self.create_publisher(PoseArray, 'cad_waypoints', 10)
        
        # Get package share directory and DXF file path
        package_share_directory = get_package_share_directory('E03P02_RI_SMS_NEA')
        self.dxf_file_path = os.path.join(package_share_directory, 'CAD', 'WAYPOINT.dxf')
        
        # Drawing positioning - move drawing in front of robot
        self.drawing_offset_x = 0.6   # Move drawing forward (in front of robot)
        self.drawing_offset_y = 0.0   # Center left/right
        self.working_height = 0.3     # Working height
        self.approach_height = 0.4    # Approach height
        
        self.get_logger().info(f'Looking for DXF file at: {self.dxf_file_path}')
        
        # Timer to publish waypoints once on startup
        self.timer = self.create_timer(2.0, self.publish_waypoints_once)
        
        self.get_logger().info('DXF Parser node started - Drawing positioned in front of robot')

    def publish_waypoints_once(self):
        """Read DXF file and publish waypoints with proper positioning"""
        waypoints = PoseArray()
        waypoints.header.stamp = self.get_clock().now().to_msg()
        waypoints.header.frame_id = "base_link"
        
        try:
            # Check if DXF file exists
            if not os.path.exists(self.dxf_file_path):
                self.get_logger().warning(f"DXF file not found at: {self.dxf_file_path}")
                self.get_logger().info("Using positioned sample waypoints instead")
                waypoints.poses = self.create_positioned_sample_waypoints()
            else:
                self.get_logger().info(f"Found DXF file, reading and positioning waypoints...")
                waypoints.poses = self.read_and_position_dxf_file()
            
            self.waypoints_pub.publish(waypoints)
            self.get_logger().info(f'Published {len(waypoints.poses)} positioned CAD waypoints')
            
        except Exception as e:
            self.get_logger().error(f"Error processing waypoints: {e}")
            waypoints.poses = self.create_positioned_sample_waypoints()
            self.waypoints_pub.publish(waypoints)
        
        # Stop the timer after publishing once
        self.timer.cancel()
    
    def read_and_position_dxf_file(self):
        """Read DXF file and position it properly in front of robot"""
        # For now, we'll use positioned sample waypoints
        # In a full implementation, you would parse the actual DXF file and apply offsets
        self.get_logger().info("Using positioned sample waypoints (DXF parsing not implemented)")
        return self.create_positioned_sample_waypoints()
    
    def create_positioned_sample_waypoints(self):
        """Create sample waypoints positioned in front of the robot"""
        # Outer rectangle waypoints (boundary) - positioned in front of robot
        outer_rect = self.create_rectangle(
            self.drawing_offset_x,  # Center X in front of robot
            self.drawing_offset_y,  # Center Y (centered)
            0.8, 0.6,              # Width, height
            self.working_height, 10 # Z height, points per side
        )
        
        # Inner circle waypoints
        inner_circle = self.create_circle(
            self.drawing_offset_x,   # Center X
            self.drawing_offset_y,   # Center Y  
            0.1,                     # Radius
            self.working_height, 16  # Z height, number of points
        )
        
        # Inner rectangle waypoints
        inner_rect = self.create_rectangle(
            self.drawing_offset_x + 0.2,  # Offset from center
            self.drawing_offset_y + 0.1,  # Offset from center
            0.2, 0.15,                    # Width, height
            self.working_height, 8        # Z height, points per side
        )
        
        # Combine all waypoints with approach heights
        all_waypoints = []
        
        # Approach to outer rectangle
        all_waypoints.append(self.create_pose(
            outer_rect[0][0], outer_rect[0][1], self.approach_height
        ))
        
        # Outer rectangle at working height
        for point in outer_rect:
            all_waypoints.append(self.create_pose(point[0], point[1], point[2]))
        
        # Approach to inner circle
        all_waypoints.append(self.create_pose(
            inner_circle[0][0], inner_circle[0][1], self.approach_height
        ))
        
        # Inner circle at working height
        for point in inner_circle:
            all_waypoints.append(self.create_pose(point[0], point[1], point[2]))
        
        # Approach to inner rectangle
        all_waypoints.append(self.create_pose(
            inner_rect[0][0], inner_rect[0][1], self.approach_height
        ))
        
        # Inner rectangle at working height
        for point in inner_rect:
            all_waypoints.append(self.create_pose(point[0], point[1], point[2]))
        
        # Return to home (in front of robot, elevated)
        all_waypoints.append(self.create_pose(
            self.drawing_offset_x, self.drawing_offset_y, self.approach_height
        ))
        
        return all_waypoints
    
    def create_rectangle(self, center_x, center_y, width, height, z, points_per_side):
        """Create rectangle waypoints with proper positioning"""
        half_w = width / 2
        half_h = height / 2
        points = []
        
        # Bottom side (right to left)
        for i in range(points_per_side):
            x = center_x + half_w - (width * i / points_per_side)
            y = center_y - half_h
            points.append((x, y, z))
        
        # Left side (bottom to top)
        for i in range(points_per_side):
            x = center_x - half_w
            y = center_y - half_h + (height * i / points_per_side)
            points.append((x, y, z))
        
        # Top side (left to right)
        for i in range(points_per_side):
            x = center_x - half_w + (width * i / points_per_side)
            y = center_y + half_h
            points.append((x, y, z))
        
        # Right side (top to bottom)
        for i in range(points_per_side):
            x = center_x + half_w
            y = center_y + half_h - (height * i / points_per_side)
            points.append((x, y, z))
        
        # Close the rectangle
        points.append(points[0])
        
        return points
    
    def create_circle(self, center_x, center_y, radius, z, num_points):
        """Create circle waypoints with proper positioning"""
        points = []
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            points.append((x, y, z))
        return points
    
    def create_pose(self, x, y, z):
        """Create a Pose message"""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = DXFParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()