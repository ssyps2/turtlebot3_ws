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
        
        self.desired_distance = 0.3     # meters
        self.desired_angle = 0.0        # rad
        
        self.angle_err_sub = self.create_subscription(Float32, "/angle_err", self.angle_chase_callback, 10)
        self.dist_err_sub = self.create_subscription(Float32, "/dist_err", self.dist_chase_callback, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
    def angle_chase_callback(self,msg:Float32):
        