#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge #add dependency
from geometry_msgs.msg import Twist

class chase_object(Node):
    def __init__(self):
        super().__init__("chase_object")
        self.get_logger().info("Chase Started")
        
        self.angle_desired = 0.0    # rad
        self.dist_desired = 0.2     # meters
        
        self.err_angle = 0.0
        self.err_dist = 0.0
        self.prev_err_angle = 0.0
        self.prev_err_dist = 0.0
        self.integral_angle = 0.0
        self.integral_dist = 0.0
        
        self.angle_sub = self.create_subscription(Float32, "/angle_measured", self.angle_chase_callback, 10)
        self.dist_sub = self.create_subscription(Float32, "/dist_measured", self.dist_chase_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
    def angle_chase_callback(self, angle_measured:Float32):
        self.err_angle = self.angle_desired - angle_measured
        
        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        while self.err_angle > np.pi:
            self.err_angle -= 2.0 * np.pi
        while self.err_angle < -np.pi:
            self.err_angle += 2.0 * np.pi
            
        Kp_angle = 2.0
        Ki_angle = 0.1
        Kd_angle = 0.01
        
        self.integral_angle += self.err_angle
        derivative_angle = self.err_angle - self.prev_err_angle
        
        if self.err_angle >= 0.08: #checking whether heading angle error within tolerence
            self.cmd_vel_angle = Kp_angle * self.err_angle + Ki_angle * self.integral_angle + Kd_angle * derivative_angle
            self.prev_err_angle = self.err_angle
            
        else:
            self.get_logger().info(f"Stopping goal heading within tolerence (angular)")
            self.cmd_vel_angle = 0.0
            
    def dist_chase_callback(self, dist_measured:Float32):
        self.err_dist = self.dist_desired - dist_measured
            
        Kp_dist = 0.4
        Ki_dist = 0.1
        Kd_dist = 0.08
        
        self.integral_dist += self.err_dist
        derivative_dist = self.err_dist - self.prev_err_dist
        
        if self.err_dist >= 0.05:
            self.cmd_vel_dist = Kp_dist * self.err_dist + Ki_dist * self.integral_dist + Kd_dist * derivative_dist
            self.prev_err_dist = self.err_dist
            
        else:
            self.get_logger().info(f"Stopping goal heading within tolerence (linear)")
            self.cmd_vel_dist = 0.0
            
    def twist_pub(self):
        cmd = Twist()
        cmd.linear.x = self.cmd_vel_dist
        cmd.angular.z = self.cmd_vel_angle
        self.vel_pub.publish(cmd)
        
        
def main():
    rclpy.init()
    chase_object_node = chase_object()

    while rclpy.ok():
        rclpy.spin_once(chase_object_node)

    chase_object_node.destroy_node()
    rclpy.shutdown()