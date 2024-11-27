import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSReliabilityPolicy
import numpy as np


class getObjectRange(Node):
    def __init__(self):
        super().__init__("getObjectRange")
        self.get_logger().info("getObjectRange Started")
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.get_Lidar_msg = self.create_subscription(LaserScan,'/scan',self.Lidar_Scan_callback,qos_profile)
        self.vector_pub = self.create_publisher(Float64MultiArray,'/obstacle_vector',10)

    
    def Lidar_Scan_callback(self,msg:LaserScan): 
        ### Step1, Abstract valued points measured from Lidar ###
        detect_radius = 0.7

        Lidar_ranges = np.array(msg.ranges)
        
        Lidar_angle_sequence = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        Lidar_angle_sequence = Lidar_angle_sequence[Lidar_ranges <= detect_radius]
        
        Lidar_ranges = Lidar_ranges[Lidar_ranges < detect_radius]
        
        if Lidar_ranges.size != 0:
            Lidar_closest_dist_index = np.argmin(Lidar_ranges)
            Lidar_closest_dist = Lidar_ranges[Lidar_closest_dist_index]
            
            Lidar_closest_angle = Lidar_angle_sequence[Lidar_closest_dist_index]
                
            if Lidar_closest_angle > np.pi:
                Lidar_closest_angle = Lidar_closest_angle - 2.0*np.pi
                
            self.get_logger().info(f'LIDAR Scan callback: distance = {str(Lidar_closest_dist)}')
            self.get_logger().info(f'LIDAR Scan callback: angle = {str(Lidar_closest_angle / np.pi * 180)}')
        
        else:
            Lidar_closest_dist = 10.0  # impossible value
            Lidar_closest_angle = 0.0
            self.get_logger().info('No Wall in Front')
            
        ### Step2, publish the vector to the nearest points ###
        vector = Float64MultiArray()
        vector.data = (float(Lidar_closest_dist),float(Lidar_closest_angle / np.pi * 180))
        self.vector_pub.publish(vector)
        
        
def main():
    rclpy.init()
    getObjectRange_node = getObjectRange()

    while rclpy.ok():
        rclpy.spin_once(getObjectRange_node)
    
    getObjectRange_node.destroy_node()
    rclpy.shutdown()