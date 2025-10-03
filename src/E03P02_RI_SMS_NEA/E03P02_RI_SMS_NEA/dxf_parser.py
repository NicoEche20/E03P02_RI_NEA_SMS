#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import ezdxf
from ament_index_python.packages import get_package_share_directory
import os
import math

class DXFWaypointParser(Node):
    def __init__(self):
        super().__init__('dxf_waypoint_parser')
        
        # Declare parameters
        self.declare_parameter('dxf_file', 'WAYPOINT.DXF')
        self.declare_parameter('package_name', 'E03P02_RI_SMS_NEA')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('sampling_distance', 0.05)  # meters between waypoints
        self.declare_parameter('z_height', 0.0)  # default z coordinate
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        # Get parameters
        dxf_filename = self.get_parameter('dxf_file').value
        package_name = self.get_parameter('package_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sampling_distance = self.get_parameter('sampling_distance').value
        self.z_height = self.get_parameter('z_height').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.pose_array_pub = self.create_publisher(PoseArray, 'waypoint_poses', 10)
        self.path_pub = self.create_publisher(Path, 'waypoint_path', 10)
        
        # Build path to DXF file
        try:
            package_share_dir = get_package_share_directory(package_name)
            dxf_path = os.path.join(package_share_dir, 'CAD', dxf_filename)
        except:
            # Fallback to local path if package not found
            dxf_path = os.path.join(os.getcwd(), 'CAD', dxf_filename)
            self.get_logger().warn(f'Package share directory not found, using: {dxf_path}')
        
        self.get_logger().info(f'Loading DXF file from: {dxf_path}')
        
        # Parse DXF file
        self.waypoints = self.parse_dxf(dxf_path)
        
        if self.waypoints:
            self.get_logger().info(f'Successfully parsed {len(self.waypoints)} waypoints')
            # Create timer to publish waypoints periodically
            self.timer = self.create_timer(1.0 / publish_rate, self.publish_waypoints)
        else:
            self.get_logger().error('No waypoints found in DXF file')
    
    def parse_dxf(self, dxf_path):
        """Parse DXF file and extract waypoints from contours"""
        waypoints = []
        
        try:
            # Load DXF file
            doc = ezdxf.readfile(dxf_path)
            msp = doc.modelspace()
            
            # Extract entities
            for entity in msp:
                if entity.dxftype() == 'LINE':
                    waypoints.extend(self.process_line(entity))
                elif entity.dxftype() == 'LWPOLYLINE':
                    waypoints.extend(self.process_polyline(entity))
                elif entity.dxftype() == 'POLYLINE':
                    waypoints.extend(self.process_polyline(entity))
                elif entity.dxftype() == 'CIRCLE':
                    waypoints.extend(self.process_circle(entity))
                elif entity.dxftype() == 'ARC':
                    waypoints.extend(self.process_arc(entity))
                elif entity.dxftype() == 'SPLINE':
                    waypoints.extend(self.process_spline(entity))
            
            # Remove duplicate consecutive points
            waypoints = self.remove_duplicates(waypoints)
            
            self.get_logger().info(f'Extracted entities from DXF: {len(waypoints)} points')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing DXF file: {str(e)}')
            return []
        
        return waypoints
    
    def process_line(self, line):
        """Extract waypoints from a line entity"""
        start = line.dxf.start
        end = line.dxf.end
        return self.interpolate_points(start, end)
    
    def process_polyline(self, polyline):
        """Extract waypoints from a polyline entity"""
        waypoints = []
        points = list(polyline.points())
        
        for i in range(len(points) - 1):
            waypoints.extend(self.interpolate_points(points[i], points[i + 1]))
        
        # Close the loop if polyline is closed
        if polyline.is_closed and len(points) > 0:
            waypoints.extend(self.interpolate_points(points[-1], points[0]))
        
        return waypoints
    
    def process_circle(self, circle):
        """Extract waypoints from a circle entity"""
        center = circle.dxf.center
        radius = circle.dxf.radius
        
        # Calculate number of points based on circumference and sampling distance
        circumference = 2 * math.pi * radius
        num_points = max(int(circumference / self.sampling_distance), 8)
        
        waypoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            waypoints.append((x, y))
        
        return waypoints
    
    def process_arc(self, arc):
        """Extract waypoints from an arc entity"""
        center = arc.dxf.center
        radius = arc.dxf.radius
        start_angle = math.radians(arc.dxf.start_angle)
        end_angle = math.radians(arc.dxf.end_angle)
        
        # Handle angle wrapping
        if end_angle < start_angle:
            end_angle += 2 * math.pi
        
        # Calculate arc length and number of points
        arc_length = radius * (end_angle - start_angle)
        num_points = max(int(arc_length / self.sampling_distance), 2)
        
        waypoints = []
        for i in range(num_points):
            angle = start_angle + (end_angle - start_angle) * i / (num_points - 1)
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            waypoints.append((x, y))
        
        return waypoints
    
    def process_spline(self, spline):
        """Extract waypoints from a spline entity"""
        waypoints = []
        try:
            # Sample the spline at regular intervals
            flattened = list(spline.flattening(self.sampling_distance))
            for point in flattened:
                waypoints.append((point[0], point[1]))
        except:
            # Fallback to control points if flattening fails
            for point in spline.control_points:
                waypoints.append((point[0], point[1]))
        
        return waypoints
    
    def interpolate_points(self, start, end):
        """Interpolate points between start and end based on sampling distance"""
        start_x, start_y = start[0], start[1]
        end_x, end_y = end[0], end[1]
        
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        num_points = max(int(distance / self.sampling_distance), 2)
        
        waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * (end_x - start_x)
            y = start_y + t * (end_y - start_y)
            waypoints.append((x, y))
        
        return waypoints
    
    def remove_duplicates(self, waypoints, tolerance=1e-6):
        """Remove consecutive duplicate waypoints"""
        if not waypoints:
            return []
        
        filtered = [waypoints[0]]
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - filtered[-1][0]
            dy = waypoints[i][1] - filtered[-1][1]
            if math.sqrt(dx**2 + dy**2) > tolerance:
                filtered.append(waypoints[i])
        
        return filtered
    def publish_waypoints(self):
        """Publish waypoints as PoseArray and Path messages"""
        if not self.waypoints:
            return
        
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.frame_id
        
        # Create Path message
        path = Path()
        path.header = pose_array.header
        
        for i, (x, y) in enumerate(self.waypoints):
            # Create pose
            pose = Pose()
            pose.position.x = (x * 0.001)*1.7+0.2
            pose.position.y = (y * 0.001) *1.7
            pose.position.z = self.z_height
            
            # Calculate orientation (tangent to path)
            if i < len(self.waypoints) - 1:
                next_x, next_y = self.waypoints[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                # Last point: use orientation from previous segment
                prev_x, prev_y = self.waypoints[i - 1]
                yaw = math.atan2(y - prev_y, x - prev_x)
            
            # Convert yaw to quaternion
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            
            pose_array.poses.append(pose)
            
            # Add to path
            pose_stamped = PoseStamped()
            pose_stamped.header = pose_array.header
            pose_stamped.pose = pose
            path.poses.append(pose_stamped)
        
        # FIX: Use the correct variable names
        self.pose_array_pub.publish(pose_array)  # Changed from waypoints_msg to pose_array
        self.path_pub.publish(path)  # Also publish the path
        
        self.get_logger().info(f'Published {len(pose_array.poses)} CAD waypoints')  # Fixed variable name
        
        # STOP publishing after first time
        self.timer.cancel()  # Stop the timer
        self.get_logger().info('DXF waypoints published once - stopping publisher')

def main(args=None):
    rclpy.init(args=args)
    node = DXFWaypointParser()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()