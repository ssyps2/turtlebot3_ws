import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSReliabilityPolicy
import numpy as np


class getObjectRange(Node):
    def __init__(self):
        #format: .fcn() or .instance
        super().__init__("getObjectRange")
        self.get_logger().info("getObjectRange Started")
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.get_Lidar_msg = self.create_subscription(LaserScan,'/scan',self.Lidar_Scan_callback,qos_profile)
        self.vector_pub = self.create_publisher(Float64MultiArray,'/obstacle_vector',10)


    #@Argument:
    # self: Pass the created instance itself, so that the defined method here can have access to other method in the class 
    # msg:Pose msg here is a name of variance, and :Pose indicate the type of variable
    
    def Lidar_Scan_callback(self,msg:LaserScan): 

        #### Step1, Abstract valued points measured from Lidar based on the distance and FoV ###
        detect_radius = 0.3
        detect_FoV = 300 / 180 * np.pi
        Lidar_ranges = np.array(msg.ranges)
        
        ## Filter the range points according to their distances ####
        Lidar_angle_sequence = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        Lidar_angle_sequence = Lidar_angle_sequence[Lidar_ranges <= detect_radius]
        
        Lidar_ranges = Lidar_ranges[Lidar_ranges < detect_radius]
        

        ##### Filter the range points again by checking whether they are in desired FoV  #####3
        if Lidar_ranges.size != 0: #If there are points within detect distance

            # Pick the closest points
            Lidar_closest_dis_index = np.argmin(Lidar_ranges)
            Lidar_closest_dis = Lidar_ranges[Lidar_closest_dis_index]
            Lidar_closest_angle = Lidar_angle_sequence[Lidar_closest_dis_index]
            
            
            if (Lidar_closest_angle > detect_FoV*0.5 and Lidar_closest_angle < (2*np.pi - detect_FoV*0.5)):
                
                Lidar_closest_dis = 1.0
                Lidar_closest_angle = 0
                self.get_logger().info('No Object')
            else:
                
                if Lidar_closest_angle > np.pi:
                    Lidar_closest_angle = Lidar_closest_angle - 2*np.pi
                    
                self.get_logger().info(f'LIDAR Scan callback: distance = {str(Lidar_closest_dis)}')
                self.get_logger().info(f'LIDAR Scan callback: angle = {str(Lidar_closest_angle/np.pi *180)}')     
                      
        else:
            Lidar_closest_dis = 10.0
            Lidar_closest_angle = 0
            self.get_logger().info('No Object')


        ### Step2, Publish the vector to the nearest points ###
        vector = Float64MultiArray()
        vector.data = (float(Lidar_closest_dis), float(Lidar_closest_angle))

        self.vector_pub.publish(vector)
        

        
def main():
    rclpy.init()
    get_object_range_node = getObjectRange()

    while rclpy.ok():
        rclpy.spin_once(get_object_range_node)

    get_object_range_node.destroy_node()
    rclpy.shutdown()