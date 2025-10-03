#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from example_interfaces.srv import Trigger
import time

class CADToExecution(Node):
    def __init__(self):
        super().__init__('cad_to_execution')
        
        # State machine states
        self.STATE_IDLE = 0
        self.STATE_PARSING = 1
        self.STATE_PLANNING = 2
        self.STATE_EXECUTING = 3
        self.STATE_COMPLETE = 4
        
        self.current_state = self.STATE_IDLE
        
        # Subscribers
        self.waypoints_subscriber = self.create_subscription(
            PoseArray,
            'cad_waypoints',
            self.waypoints_callback,
            10
        )
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.desired_pose_publisher = self.create_publisher(
            Pose,
            'desired_pose',
            10
        )
        
        self.waypoints_for_planner_publisher = self.create_publisher(
            PoseArray,
            'desired_waypoints',
            10
        )
        
        # Services
        self.execute_service = self.create_service(
            Trigger,
            'execute_cad_design',
            self.execute_callback
        )
        
        # Internal state
        self.cad_waypoints = None
        self.current_joint_state = None
        self.current_trajectory = None
        
        self.get_logger().info('CAD to Execution node started')
        
    def waypoints_callback(self, msg):
        """Receive waypoints from DXF parser"""
        self.cad_waypoints = msg
        self.get_logger().info(f"Received {len(msg.poses)} CAD waypoints")
        
    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg
        
    def execute_callback(self, request, response):
        """Service callback to execute complete CAD design"""
        try:
            if self.current_state != self.STATE_IDLE:
                response.success = False
                response.message = "System busy, cannot execute now"
                return response
            
            # Start execution pipeline
            self.execute_pipeline()
            
            response.success = True
            response.message = "CAD execution pipeline started"
            
        except Exception as e:
            response.success = False
            response.message = f"Execution failed: {str(e)}"
            
        return response
    
    def execute_pipeline(self):
        """Execute the complete CAD to execution pipeline"""
        self.get_logger().info("Starting CAD execution pipeline...")
        
        # Step 1: Parse DXF (trigger via service)
        self.get_logger().info("Step 1: Parsing DXF file...")
        self.current_state = self.STATE_PARSING
        
        # In a real implementation, you'd call the DXF parser service here
        # For now, we assume waypoints are already received
        
        if not self.cad_waypoints:
            self.get_logger().error("No CAD waypoints available")
            self.current_state = self.STATE_IDLE
            return
        
        # Step 2: Send waypoints to trajectory planner
        self.get_logger().info("Step 2: Sending waypoints to trajectory planner...")
        self.current_state = self.STATE_PLANNING
        
        self.waypoints_for_planner_publisher.publish(self.cad_waypoints)
        
        # Step 3: Wait a bit for trajectory planning, then execute
        self.get_logger().info("Step 3: Starting trajectory execution...")
        self.current_state = self.STATE_EXECUTING
        
        # Execute trajectory point by point
        self.execute_trajectory_points()
        
    def execute_trajectory_points(self):
        """Execute trajectory by publishing desired poses"""
        if not self.cad_waypoints:
            return
            
        self.get_logger().info(f"Executing {len(self.cad_waypoints.poses)} waypoints...")
        
        for i, pose in enumerate(self.cad_waypoints.poses):
            self.get_logger().info(f"Executing waypoint {i+1}/{len(self.cad_waypoints.poses)}")
            
            # Publish desired pose (this will trigger IK and joint commands)
            self.desired_pose_publisher.publish(pose)
            
            # Wait for execution (simplified - in real system, you'd check for completion)
            time.sleep(2.0)  # Adjust based on your system
        
        self.get_logger().info("CAD execution complete!")
        self.current_state = self.STATE_COMPLETE
        
        # Return to idle after completion
        self.current_state = self.STATE_IDLE

def main(args=None):
    rclpy.init(args=args)
    
    cad_executor = CADToExecution()
    
    try:
        rclpy.spin(cad_executor)
    except KeyboardInterrupt:
        cad_executor.get_logger().info('CAD to Execution node shutting down...')
    finally:
        cad_executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()