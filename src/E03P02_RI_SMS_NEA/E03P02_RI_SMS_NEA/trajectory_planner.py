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
            PoseArray, 'waypoint_poses', self.waypoints_callback, 10
        )
        
        # Publish desired poses for IK
        self.desired_pose_pub = self.create_publisher(Pose, 'desired_pose', 10)
        
        self.current_waypoints = []
        self.trajectory_points = []
        self.current_point_index = 0
        self.last_publish_time = 0
        
        # Trajectory parameters
        self.point_duration = 0.2  # 5 Hz - más lento para visualización clara
        self.interpolation_points = 20  # Puntos entre waypoints
        
        # Timer for smooth trajectory execution
        self.timer = self.create_timer(self.point_duration, self.execute_trajectory)
        
        self.get_logger().info('Trajectory Planner node started - QUINTIC SPLINE mode')

    def waypoints_callback(self, msg):
        """Receive CAD waypoints and create trajectory"""
        if not msg.poses:
            self.get_logger().warn('Received empty waypoints')
            return
            
        self.current_waypoints = msg.poses
        self.get_logger().info(f'Received {len(self.current_waypoints)} waypoints')
        
        # Create trajectory using QUINTIC SPLINE per segment
        self.trajectory_points = self.create_quintic_trajectory_per_segment(self.current_waypoints)
        self.current_point_index = 0
        self.last_publish_time = time.time()
        
        self.get_logger().info(f'Created quintic trajectory with {len(self.trajectory_points)} points')

    def create_quintic_trajectory_per_segment(self, waypoints):
        """
        Quintic spline aplicado por segmento para mantener esquinas rectas.
        Procesa cada lado del rectángulo independientemente.
        """
        if len(waypoints) < 2:
            return waypoints
            
        trajectory_points = []
        
        # Procesar cada segmento independientemente
        for i in range(len(waypoints) - 1):
            start_pose = waypoints[i]
            end_pose = waypoints[i + 1]
            
            # Detectar tipo de movimiento
            z_diff = abs(end_pose.position.z - start_pose.position.z)
            xy_diff = math.sqrt(
                (end_pose.position.x - start_pose.position.x)**2 + 
                (end_pose.position.y - start_pose.position.y)**2
            )
            
            # Usar menos puntos para movimientos verticales (approach)
            num_points = 10 if z_diff > xy_diff else self.interpolation_points
            
            # Crear interpolación quintic para este segmento
            segment_points = self.interpolate_segment_quintic(
                start_pose, end_pose, num_points
            )
            
            trajectory_points.extend(segment_points)
        
        return trajectory_points

    def interpolate_segment_quintic(self, start_pose, end_pose, num_points):
        """
        Interpola un segmento usando polinomio quintic (5to grado).
        Esto da control sobre posición, velocidad y aceleración.
        """
        segment_points = []
        
        for i in range(num_points):
            # Parámetro t de 0 a 1
            t = i / float(num_points)
            
            # QUINTIC POLYNOMIAL (5to grado)
            # s(t) = 6t^5 - 15t^4 + 10t^3
            # Esta función tiene:
            # - s(0) = 0, s(1) = 1 (posición)
            # - s'(0) = 0, s'(1) = 0 (velocidad cero en extremos)
            # - s''(0) = 0, s''(1) = 0 (aceleración cero en extremos)
            
            t2 = t * t
            t3 = t2 * t
            t4 = t3 * t
            t5 = t4 * t
            
            # Función quintic suave
            s = 6*t5 - 15*t4 + 10*t3
            
            # Interpolar posición usando la función quintic
            x = start_pose.position.x + s * (end_pose.position.x - start_pose.position.x)
            y = start_pose.position.y + s * (end_pose.position.y - start_pose.position.y)
            z = start_pose.position.z + s * (end_pose.position.z - start_pose.position.z)
            
            # Crear pose interpolada
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation = start_pose.orientation
            
            segment_points.append(pose)
        
        # Agregar el punto final exacto
        segment_points.append(end_pose)
        
        return segment_points

    def create_linear_trajectory(self, waypoints):
        """
        ALTERNATIVA: Interpolación lineal (para comparación).
        Descomenta esta función y cámbiala en waypoints_callback si prefieres linear.
        """
        if len(waypoints) < 2:
            return waypoints
            
        trajectory_points = []
        
        for i in range(len(waypoints) - 1):
            start_pose = waypoints[i]
            end_pose = waypoints[i + 1]
            
            # Detectar tipo de movimiento
            z_diff = abs(end_pose.position.z - start_pose.position.z)
            xy_diff = math.sqrt(
                (end_pose.position.x - start_pose.position.x)**2 + 
                (end_pose.position.y - start_pose.position.y)**2
            )
            
            num_points = 10 if z_diff > xy_diff else self.interpolation_points
            
            # Interpolación LINEAR simple
            for j in range(num_points):
                t = j / float(num_points)
                
                x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x)
                y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y)
                z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z)
                
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)
                pose.orientation = start_pose.orientation
                
                trajectory_points.append(pose)
            
            trajectory_points.append(end_pose)
        
        return trajectory_points

    def execute_trajectory(self):
        """Execute trajectory point by point"""
        if not self.trajectory_points:
            return
            
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_publish_time < self.point_duration - 0.01:
            return
            
        # Get current point and publish it
        pose = self.trajectory_points[self.current_point_index]
        self.desired_pose_pub.publish(pose)
        
        # Move to next point - NO LOOPING
        self.current_point_index += 1
        
        # Stop when trajectory is complete
        if self.current_point_index >= len(self.trajectory_points):
            self.get_logger().info('🎉 TRAJECTORY COMPLETED! Stopping execution.')
            self.trajectory_points = []  # Clear trajectory to stop execution
            return
        
        self.last_publish_time = current_time
        
        # Log progress occasionally
        if self.current_point_index % 100 == 0:  # Increased to every 100 points
            self.get_logger().info(
                f'Progress: {self.current_point_index}/{len(self.trajectory_points)} - '
                f'Pos: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Trajectory Planner shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()