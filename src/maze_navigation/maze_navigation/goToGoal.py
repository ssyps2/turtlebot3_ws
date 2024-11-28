import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSReliabilityPolicy
from collections import deque, Counter

from math import atan2, sqrt
import numpy as np
import math


class goToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        # State (for the update_Odometry code)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.robot_angle = 0.0
        
        self.states_list = (0, 1, 2, 3, 4, 5)
        self.current_state = 0

        self.err_angle = 0.0
        self.prev_err_angle = 0.0
        self.integral_angle = 0.0

        self.cmd_vel_linear = 0.0
        self.cmd_vel_angle = 0.0
        self.ref_vel_angle = 0.0

        self.set_limit = 0.4
        self.detect_radius = 0.6
        self.detect_ang_range = 60.0
        self.turn_record_flag = 0

        self.wall_front_dist = 0.0
        self.wall_front_ang = 0.0
        self.wall_side_dist = 0.0
        self.wall_side_ang = 0.0

        self.recog_result = 0.0
        self.img_center_ang = 0.0

        # self.recog_result_queue = deque(maxlen=10)

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,qos_profile)
        self.wall_vector_subscriber = self.create_subscription(Float64MultiArray,'/obstacle_vector',self.wall_vector_callback,qos_profile)
        self.recog_result_subscriber = self.create_subscription(Int32,'/recog_label',self.recog_result_callback,qos_profile)
        self.img_center_subscriber = self.create_subscription(Float64,'/img_center_angle',self.img_center_callback,qos_profile)

        self.cmd_vel_publisher = self.create_publisher(Twist,'/cmd_vel',10)
    

    def odom_callback(self, msg):
        self.update_Odometry(msg)
        self.move_to_goal()


    def wall_vector_callback(self, msg):
        self.wall_front_dist = msg.data[0]
        self.wall_front_ang  = msg.data[1]/np.pi*180    # in deg
        self.wall_side_dist  = msg.data[2]
        self.wall_side_ang   = msg.data[3]/np.pi*180    # in deg


    def recog_result_callback(self, msg):
        self.recog_result = msg.data


    def img_center_callback(self, msg):
        self.img_center_ang = msg.data*np.pi/180  # in rad


    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        if self.globalAng > np.pi:
            self.globalAng = self.globalAng - 2.0*np.pi
    

    def move_to_goal(self):
        # Angular PID parameters
        Kp_angle = 0.8
        Ki_angle = 0.0
        Kd_angle = 0.0
       
        ####### States Machine #######
        self.label_dic = {
                            "Empty": 0,
                            "left" : 1,
                            "right": 2,
                            "do not enter": 3,
                            "stop": 4,
                            "goal": 5
                        }
        
        #### Empty
        if self.current_state == 0:
            # Go straight
            self.cmd_vel_linear = 0.1
            if(self.wall_front_ang >= -15.0) & (self.wall_front_ang <= 15.0):
                # Go straight and track the color sign angularly (within lidar detect_radius)
                if (self.wall_front_dist >= self.set_limit) & (self.wall_front_dist <= self.detect_radius):
                    self.cmd_vel_linear = 0.1
                    self.robot_angle = 0.0
                    self.ref_vel_angle = self.img_center_ang
                    self.current_state = 0
                # Stop and track the color sign angularly
                elif self.wall_front_dist < self.set_limit:
                    self.cmd_vel_linear = 0.0
                    self.robot_angle = 0.0
                    self.ref_vel_angle = self.img_center_ang

                    # when it runs to corners
                    if self.wall_side_dist < self.set_limit:
                        self.cmd_vel_linear = -0.08
                        self.get_logger().info("Going back")

                    # ## Average recognition result by using queue
                    # self.recog_result_queue.append(self.recog_result)
                    # count = Counter(self.recog_result_stack) # Count the occurrences of each number in the queue

                    # ## ---- STATE ONLY UPDATE HERE ---- ##
                    # self.current_state = count.most_common(1)[0][0]
                    self.current_state = self.recog_result

                    # After the state is updated, stop rotating
                    if self.current_state != 0:
                        self.get_logger().info("State Changed")
                        self.robot_angle = 0.0
                        self.ref_vel_angle = 0.0
                    else:
                        self.get_logger().info("State still 0") # should swing around


                # Go straight if no wall in front
                elif self.wall_front_dist > self.detect_radius:
                    self.cmd_vel_linear = 0.1
                    self.ref_vel_angle = 0.0
                    self.robot_angle = 0.0
                    self.current_state = 0
            
            # Avoid hitting wall from right side
            if (self.wall_side_ang < -15.0) & (self.wall_side_ang > -self.detect_ang_range):
                if self.wall_side_dist <= self.set_limit:
                    self.ref_vel_angle = (self.wall_side_ang + 90.0) * np.pi / 180
            
            ## Avoid hitting wall from left side
            if (self.wall_side_ang > 15.0) & (self.wall_side_ang < self.detect_ang_range):
                if self.wall_side_dist <= self.set_limit:
                    self.ref_vel_angle = (self.wall_side_ang - 90.0) * np.pi / 180
                
        #### Go Left
        elif self.current_state == 1:
            self.cmd_vel_linear = 0.0
            # Record the initial angle
            if self.turn_record_flag == 0:
                self.robot_angle = self.globalAng   # in rad
                self.ref_vel_angle = self.globalAng + np.pi*0.5
                self.turn_record_flag = 1
            # Update robot angle
            elif self.turn_record_flag == 1:
                self.robot_angle = self.globalAng
                # when the error is small enough
                if abs(self.err_angle) < (5*np.pi/180):
                    self.current_state = 0
                    self.turn_record_flag = 0

        #### Go Right
        elif self.current_state == 2:
            self.cmd_vel_linear = 0.0
            # Record the initial angle
            if self.turn_record_flag == 0:
                self.robot_angle = self.globalAng   # in rad
                self.ref_vel_angle = self.globalAng - np.pi*0.5
                self.turn_record_flag = 1
            # Update robot angle
            elif self.turn_record_flag == 1:
                self.robot_angle = self.globalAng
                # when the error is small enough
                if abs(self.err_angle) < (5*np.pi/180):
                    self.current_state = 0
                    self.turn_record_flag = 0
        
        #### Make U Turn
        elif self.current_state == 3:
            self.cmd_vel_linear = 0.0
            # Record the initial angle
            if self.turn_record_flag == 0:
                self.robot_angle = self.globalAng   # in rad
                self.ref_vel_angle = self.globalAng + np.pi
                self.turn_record_flag = 1
            # Update robot angle
            elif self.turn_record_flag == 1:
                self.robot_angle = self.globalAng
                # when the error is small enough
                if abs(self.err_angle) < (5*np.pi/180):
                    self.current_state = 0
                    self.turn_record_flag = 0
            
        #### Stop
        elif self.current_state == 4:
            # self.cmd_vel_linear = 0.0
            # self.ref_vel_angle = 0.0
            self.current_state = 2

        #### Goal
        elif self.current_state == 5:
            self.cmd_vel_linear = 0.0
            self.ref_vel_angle = 0.0
        

        #### Angular PID
        while self.ref_vel_angle > np.pi:
            self.ref_vel_angle -= 2.0 * np.pi
        while self.ref_vel_angle < -np.pi:
            self.ref_vel_angle += 2.0 * np.pi

        self.err_angle = self.ref_vel_angle - self.robot_angle  # in rad

        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        while self.err_angle > np.pi:
            self.err_angle -= 2.0 * np.pi
        while self.err_angle < -np.pi:
            self.err_angle += 2.0 * np.pi

        self.integral_angle += self.err_angle

        integral_angle_limit = 0.6
        if np.abs(self.integral_angle) > integral_angle_limit:
            self.integral_angle = np.sign(self.integral_angle)* integral_angle_limit
 
        derivative_angle = self.err_angle - self.prev_err_angle

        self.cmd_vel_angle = Kp_angle * self.err_angle #+ Ki_angle * self.integral_angle + Kd_angle * derivative_angle

        # Output limitation on the angular velocity
        if np.abs(self.cmd_vel_angle) > 0.8:
            self.cmd_vel_angle = np.sign(self.cmd_vel_angle) * 0.8

        self.prev_err_angle = self.err_angle


        self.get_logger().info(f"Recognized State: {self.recog_result}")
        self.get_logger().info(f"Current State: {self.current_state}")

        # self.get_logger().info(f'Global Angle in deg: {self.globalAng/np.pi*180}')
        self.get_logger().info(f'Robot Angle in deg: {self.robot_angle/np.pi*180}')
        self.get_logger().info(f'Reference Angle in deg: {self.ref_vel_angle/np.pi*180}')
        self.get_logger().info(f'Img Center Angle in deg: {self.img_center_ang/np.pi*180}')
        self.get_logger().info(f'Linear Vel: {self.cmd_vel_linear}')

        self.get_logger().info(f'Wall Front Dist: {self.wall_front_dist}')

        self.get_logger().info(f"angle err in deg: {self.err_angle*180/np.pi}")
        
        #### Publish velocity command
        cmd = Twist()
        cmd.linear.x = self.cmd_vel_linear
        cmd.angular.z = self.cmd_vel_angle
        self.cmd_vel_publisher.publish(cmd)
            

def main(args=None):
    rclpy.init(args=args)
    goToGoal_node = goToGoal()
    goToGoal_node.get_logger().info('start running')
    
    while rclpy.ok():
        rclpy.spin_once(goToGoal_node)
        
    goToGoal_node.destroy_node()
    rclpy.shutdown()
