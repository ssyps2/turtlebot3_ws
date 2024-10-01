import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSReliabilityPolicy
import numpy as np

#Datatype package
#It import a library with corresponding datatype. The datatype can be determined by ros2 topic info /nodename

class get_object_range(Node):
    def __init__(self):
        #format: .fcn() or .instance
        super().__init__("get_object_range") #assign node name
        qos_profile=QoSProfile(depth=10)
        qos_profile.reliability=QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability=QoSDurabilityPolicy.VOLATILE

        self.get_Lidar_msg=self.create_subscription(LaserScan,'/scan',self.Lidar_Scan_callback,qos_profile) #return a subscribe instance
        self.get_camera_msg=self.create_subscription(Int32,'/object_x',self.camera_coordinate_callback,qos_profile)
       
    #@Argument:
    # self: Pass the created instance itself, so that the defined method here can have access to other method in the class 
    # msg:Pose msg here is a name of variance, and :Pose indicate the type of variable
        self.x_coordinate=float()
    def Lidar_Scan_callback(self,msg:LaserScan): 
        #Preprocess the lidar data
        #self.get_logger().info(f"maximum angle: {msg.scan_time}")
        #self.get_logger().info(f"{self.x_coordinate}")
        self.get_logger().info(f"{msg.angle_min}")
        self.get_logger().info(f"{msg.angle_max}")
        
        if self.x_coordinate!=800:
            angle_from_camera=(((self.x_coordinate-160)/320)*62.2)*np.pi/180
            sequence_num=(angle_from_camera+180-msg.angle_min)//msg.angle_increment - 1
            #angle_increment=1.478 degree
            #+-5 increments
            object_distance_dataset=msg.ranges[sequence_num-5:sequence_num+5+1]

            object_distance_dataset_filtered=object_distance_dataset[(0.1<object_distance_dataset & object_distance_dataset<2)]

    def camera_coordinate_callback(self,msg:Int32):
         self.x_coordinate=float(msg.data)
         
         

        

def main(args=None):
        rclpy.init(args=args)
        node=get_object_range() #create the node
        rclpy.spin(node) #hold the node running
        rclpy.shutdown()
       