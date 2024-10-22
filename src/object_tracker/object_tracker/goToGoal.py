import rclpy
from rclpy.node import Node
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
        
        self.object_vector = None
        # 0:find_goal  1:follow_wall_cw  2:follow_wall_ccw  3:avoid_obstacles
        self.states_list = (0, 1, 2, 3)
        self.current_state = 0
        
        self.current_goal_idx = 0
        self.goal_tolerance = 0.05  # 5cm tolerance
        self.waypoints = [(1.5, 0), (1.5, 1.4), (0, 1.4)]
        
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.object_subscriber = self.create_subscription(Float64MultiArray,'/object_vector',self.object_callback,10)
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

        #We subtract the initial values
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
        
        # If close to the goal, stop and move to the next goal
        # if distance_to_goal < self.goal_tolerance:
        #     self.stop_and_wait()
        #     self.current_goal_idx += 1
        #     if(self.current_goal_idx == 3): # End
        #         self.get_logger().info('End and Exited')
        #         exit(0)
        #     return
        
        # Adjust based on object vector if there's an obstacle nearby
        if self.object_vector:
            obj_x, obj_y = self.object_vector
            if sqrt(obj_x**2 + obj_y**2) < 0.2:  # Obstacle within 20cm
                angle_to_goal += 1.0  # Example adjustment to avoid obstacle
        
        # Publish velocity command to move towards the goal
        cmd = Twist()
        cmd.linear.x = 0.5  # Set forward velocity
        cmd.angular.z = angle_to_goal  # Set angular velocity to turn to the goal
        
        self.cmd_vel_publisher.publish(cmd)
    
    def stop_and_wait(self):
        # Stop and wait for 2 seconds
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(2)



def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = goToGoal()
    
    while rclpy.ok():
        rclpy.spin_once(go_to_goal_node)
        
    go_to_goal_node.destroy_node()
    rclpy.shutdown()
