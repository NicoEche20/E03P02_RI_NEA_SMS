#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
# system library
import sys

class TurtleMoveNodeNEA(Node): # Redefine node class
    def __init__(self):
        super().__init__("turtle_move_node_nea") # Redefine node name

        # publisher obj (msg_type, topic_name, queue==buffer)
        self.cmd_pub = self.create_publisher(Twist,'/turtle1/cmd_vel',10)

        # create a timer function to send msg
        timer_period = 0.5 # in [s]
        self.timer = self.create_timer(timer_period,self.timer_callback)

        # create a topic subscriber
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.pose_subs = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,1)
 
    def timer_callback(self):
        # def twist variable
        cmd_vel = Twist()
        # msg
        cmd_vel.linear.x = 0.5  #float(sys.argv[1])#
        cmd_vel.angular.z = 1.0 #float(sys.argv[2])#
        self.cmd_pub.publish(cmd_vel)

    def pose_callback(self,data):
        msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x,data.y,data.theta)
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMoveNodeNEA() # object definition (creation)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()