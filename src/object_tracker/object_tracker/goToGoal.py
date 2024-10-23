import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

from math import atan2, sqrt
import time
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
        self.clock = Clock()
        
        self.object_vector = None
        # 0:find_goal  1:follow_wall_cw  2:follow_wall_ccw  3:avoid_obstacles
        self.states_list = (0, 1, 2, 3)
        self.current_state = 0
        
        self.current_goal_idx = 0
        self.goal_tolerance = 0.02  # 2cm tolerance
        self.waypoints = [(1.5, 0), (1.5, 1.4), (0, 1.4)]
        #self.waypoints = [(10, 0), (1, 1), (0, 1)]

        self.err_angle = 0.0
        self.prev_err_angle = 0.0
        self.integral_angle = 0.0
        self.cmd_vel_angle = 0.0
        self.cmd_vel_linear = 0.0
        
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.object_subscriber = self.create_subscription(Float64MultiArray,'/obstacle_vector',self.object_callback,10)
        self.cmd_vel_publisher = self.create_publisher(Twist,'/cmd_vel',10)
    
    def odom_callback(self, msg):
        self.update_Odometry(msg)
        self.move_to_goal()

    def object_callback(self, msg):
        self.object_vector = msg

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
    
        self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))
    
    def move_to_goal(self):
        # Get the current goal
        goal_x, goal_y = self.waypoints[self.current_goal_idx]
        
        # Calculate distance and angle to the goal
        robot_x = self.globalPos.x
        robot_y = self.globalPos.y
        distance_to_goal = sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        angle_to_goal = atan2(goal_y - robot_y, goal_x - robot_x)

        # Angular PID parameters
        Kp_angle = 1.5
        Ki_angle = 0.0
        Kd_angle = 0.0

       
        ## States Machine ##
        if self.object_vector:
            obj_dist = self.object_vector.data[0]
            obj_angle = self.object_vector.data[1]

            # Judge the angle between object_vector and obstacle vector
            rotate_matrix = np.array([[np.cos(self.globalAng), -np.sin(self.globalAng)],
                            [np.sin(self.globalAng),  np.cos(self.globalAng)]])
            
            obj_vec_robo = np.array([[np.cos(obj_angle)],
                            [np.sin(obj_angle)]])
            
            obj_vec_global = rotate_matrix @ obj_vec_robo
            goal_vec_global = np.array([[goal_x - robot_x],
                            [goal_y - robot_y]])
            
            angle_dot_product = np.dot(obj_vec_global.T, goal_vec_global)
            self.get_logger().info(f"Angle Judge: {angle_dot_product}")

            self.get_logger().info(f"Progress {self.current_goal_idx}")

            # If there's no obstacle: go to goal
            detect_threshold = 0.3

            # from following path to following wall
            if (self.current_state == 0 and obj_dist < detect_threshold) :
                if obj_angle > 0:
                    self.current_state = 1 # obj at left side, turn clockwise
                else:
                    self.current_state = 2  # obj at right side, turn counter-clockwise

            # from following the wall to following the path
            if (self.current_state != 0 and (obj_dist > detect_threshold*1.2 or angle_dot_product < 0)):
                self.current_state = 0


            if self.current_state == 0:
                
                robot_orien_vec = np.array([[np.cos(self.globalAng)],
                            [np.sin(self.globalAng)]])
        
                robot_orien_vec = np.vstack((robot_orien_vec,[0]))
                goal_orien_vec = np.vstack((goal_vec_global,[0]))

                robot_orien_norm = robot_orien_vec / np.linalg.norm(robot_orien_vec)
                goal_orien_norm = goal_orien_vec / np.linalg.norm(goal_orien_vec)

                dot_product = np.clip(np.dot(robot_orien_norm.T, goal_orien_norm), -1.0, 1.0)

                angle_difference_mag = math.acos(dot_product)
                sign = np.sign( (np.cross(robot_orien_norm.flatten(), goal_orien_norm.flatten()))[2] )

                self.err_angle = sign*angle_difference_mag


                #self.err_angle = angle_to_goal - self.globalAng
                
                self.cmd_vel_linear = 0.12

                if np.abs(self.err_angle) > np.pi/4:
                     self.cmd_vel_linear = 0.0
                
                #self.get_logger().info(f"angle_difference_mag {self.err_angle}")
                self.get_logger().info(f"go to goal")

            elif self.current_state == 1:
                self.err_angle = obj_angle - np.pi/2
                self.cmd_vel_linear = 0.1
                self.get_logger().info(f"follow wall - cw")
            else:
                self.err_angle = obj_angle + np.pi/2
                self.cmd_vel_linear = 0.1
                self.get_logger().info(f"follow wall - ccw")
            


            # if (obj_dist > detect_threshold or np.abs(angle_to_goal-obj_angle)>np.pi/2):
            #     self.current_state = 0
            #     self.err_angle = angle_to_goal - self.globalAng
            #     self.cmd_vel_linear = 0.12
            #     self.get_logger().info(f"go to goal")

            # # If there's an obstacle nearby: follow wall
            # elif(obj_dist < detect_threshold):
            #     if(obj_angle>0):
            #         self.current_state = 1  # obj at left side, turn clockwise
            #         self.err_angle = obj_angle - np.pi/3
            #         self.cmd_vel_linear = 0.08
            #         self.get_logger().info(f"follow wall - cw")

            #     elif(obj_angle<0):
            #         self.current_state = 2  # obj at right side, turn counter-clockwise
            #         self.err_angle = obj_angle + np.pi/3
            #         self.cmd_vel_linear = 0.08
            #         self.get_logger().info(f"follow wall - ccw")

            # If it's too close to obstacle: avoid
            # if obj_dist < 0.2:
            #     self.current_state = 3


        # # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
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
        if np.abs(self.cmd_vel_angle) > 1.2:
            self.cmd_vel_angle = np.sign(self.cmd_vel_angle) * 1.2

        self.prev_err_angle = self.err_angle
        self.get_logger().info(f"angle err {self.err_angle * 180/np.pi}, cmd_vel {self.cmd_vel_angle}")

        # If close to the goal, stop and move to the next goal
        cmd = Twist()
        if distance_to_goal < self.goal_tolerance:

            # Wait at the destination
            time_stamp = self.clock.now().nanoseconds/ 1e9
            while(self.clock.now().nanoseconds/ 1e9 - time_stamp) < 3:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd)
                self.get_logger().info("Waiting at destination")


            self.current_goal_idx += 1
            if(self.current_goal_idx == 3):
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd)

                self.get_logger().info('End and Exited')
                exit(0)
        
        # Publish velocity command to move towards the goal
        cmd.linear.x = self.cmd_vel_linear
        cmd.angular.z = self.cmd_vel_angle+0.02
        
        self.cmd_vel_publisher.publish(cmd)
            



def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = goToGoal()
    
    while rclpy.ok():
        rclpy.spin_once(go_to_goal_node)
        
    go_to_goal_node.destroy_node()
    rclpy.shutdown()
