import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped

from math import atan2, sqrt
import time
import numpy as np
import math

class pub_waypoints(Node):
    def __init__(self):
        super().__init__('pub_waypoints')
        # State (for the update_Odometry code)
        self.Init = True
        self.odem_Init_Flag = False
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.mapGlobalPos = Point()

        self.object_vector = None
  
        self.current_goal_idx = 0
        self.goal_tolerance = 0.4
        self.waypoints = []
        
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.waypoints_subscriber = self.create_subscription(PointStamped,'/clicked_point',self.clicked_point_callback,10)
        self.waypoints_publisher = self.create_publisher(PoseStamped,'/goal_pose',10)
        self.initial_odem_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_odem_callback,10)
    
    def initial_odem_callback(self,msg):
        if self.odem_Init_Flag == False:
            self.init_x = msg.pose.pose.position.x
            self.init_y  = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            self.init_w  = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            
            self.odem_Init_Flag = True




    def pub_goal(self,x : float, y : float):
        msg = PoseStamped()

        # Fill in the header information
        msg.header.stamp = self.get_clock().now().to_msg()  # Use current time
        msg.header.frame_id = 'map'  # Replace with your map name or coordinate frame

        # Set the position with x = 1, y = 0 (or any other values you prefer)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # Set the orientation (e.g., facing forward in the frame)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        
        self.waypoints_publisher.publish(msg)

    
    def clicked_point_callback(self,msg):
        x = msg.point.x
        y = msg.point.y
        self.waypoints.append([x,y])
        self.get_logger().info(f'Received: "{x,y}"')
     
    
    def odom_callback(self, msg):
        if self.odem_Init_Flag == True:
            self.update_Odometry(msg)
            if len(self.waypoints) > 2:
                self.move_to_goal()
                self.get_logger().info(f'World')

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

        Mrot = np.matrix([[np.cos(self.init_w), np.sin(self.init_w)],[-np.sin(self.init_w), np.cos(self.init_w)]])

        self.mapGlobalPos.x = Mrot.item((0,0))*self.globalPos.x + Mrot.item((1,0))*self.globalPos.y + self.init_x
        self.mapGlobalPos.y = Mrot.item((0,1))*self.globalPos.x + Mrot.item((1,1))*self.globalPos.y + self.init_y

        self.globalAng = orientation - self.Init_ang + self.init_w

        self.get_logger().info('Global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng-self.init_w))
        self.get_logger().info('Transformed map pose is x:{}, y:{}, a:{}'.format(self.mapGlobalPos.x-self.init_x,self.mapGlobalPos.y-self.init_y,self.globalAng))
        self.get_logger().info(f'Mrot: "{Mrot}"')
    
    def move_to_goal(self):
        # Get the current goal
        goal_x, goal_y = self.waypoints[self.current_goal_idx]
        
        # Calculate distance and angle to the goal
        robot_x = self.mapGlobalPos.x
        robot_y = self.mapGlobalPos.y
        distance_to_goal = sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        if distance_to_goal <=  self.goal_tolerance and ( (self.current_goal_idx) < (len(self.waypoints)-1) ):
            self.current_goal_idx += 1
           
        self.pub_goal(goal_x,goal_y)
        self.get_logger().info(f'Distance to the goal: "{distance_to_goal}"')
        self.get_logger().info(f'Index: "{self.current_goal_idx}"')


def main(args=None):
    rclpy.init(args=args)
    pub_waypoints_node = pub_waypoints()
    
    while rclpy.ok():
        rclpy.spin_once(pub_waypoints_node)
        
    pub_waypoints_node.destroy_node()
    rclpy.shutdown()
