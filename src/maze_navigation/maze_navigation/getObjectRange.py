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

    
    def Lidar_Scan_callback(self, msg: LaserScan):
        ### Step1, Abstract valued points measured from Lidar ###
        detect_radius = 0.6
        detect_ang_range = 60.0 * np.pi / 180
        detect_front_range = 15.0 * np.pi / 180

        Lidar_ranges = np.array(msg.ranges)

        # Generate angles corresponding to the Lidar ranges
        Lidar_angle_sequence = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # Filter for points within the detect radius
        Lidar_angle_sequence = Lidar_angle_sequence[Lidar_ranges <= detect_radius]  # angle in rad
        Lidar_ranges = Lidar_ranges[Lidar_ranges <= detect_radius]                  # dist

        # Separate points into the two angular ranges
        within_front_range = ((Lidar_angle_sequence >= (2.0*np.pi-detect_front_range)) & (Lidar_angle_sequence <= 2.0*np.pi)) | ((Lidar_angle_sequence <= detect_front_range) & (Lidar_angle_sequence >= 0))
        within_detect_range = (((Lidar_angle_sequence >= (2.0*np.pi-detect_ang_range)) & (Lidar_angle_sequence <= 2.0*np.pi)) | ((Lidar_angle_sequence <= detect_ang_range) & (Lidar_angle_sequence >= 0))) & (~within_front_range)

        # self.get_logger().info(f'{Lidar_angle_sequence * 180 / np.pi}')

        # Process the front range (-15 to 15 degrees)
        if np.any(within_front_range):
            front_ranges = Lidar_ranges[within_front_range]
            front_angles = Lidar_angle_sequence[within_front_range]

            front_closest_index = np.argmin(front_ranges)
            front_closest_dist = front_ranges[front_closest_index]
            front_closest_angle = front_angles[front_closest_index]
        else:
            front_closest_dist = 10.0  # Impossible value
            front_closest_angle = 0.0

        # Process the detect range (-60 to -15 and 15 to 60 degrees)
        if np.any(within_detect_range):
            detect_ranges = Lidar_ranges[within_detect_range]
            detect_angles = Lidar_angle_sequence[within_detect_range]

            detect_closest_index = np.argmin(detect_ranges)
            detect_closest_dist = detect_ranges[detect_closest_index]
            detect_closest_angle = detect_angles[detect_closest_index]
        else:
            detect_closest_dist = 10.0  # Impossible value
            detect_closest_angle = np.pi

        # Normalize angles to [-pi, pi]
        if front_closest_angle > np.pi:
            front_closest_angle -= 2.0 * np.pi
        if detect_closest_angle > np.pi:
            detect_closest_angle -= 2.0 * np.pi

        # Log results
        self.get_logger().info(f'Front Range: distance = {front_closest_dist}, angle = {front_closest_angle / np.pi * 180}')
        self.get_logger().info(f'Detect Range: distance = {detect_closest_dist}, angle = {detect_closest_angle / np.pi * 180}')

        ### Step2, Publish the vectors ###
        # Front range vector and Detect range vector
        front_vector = Float64MultiArray()
        front_vector.data = (float(front_closest_dist),float(front_closest_angle),float(detect_closest_dist),float(detect_closest_angle))
        self.vector_pub.publish(front_vector)
        
        
def main():
    rclpy.init()
    getObjectRange_node = getObjectRange()

    while rclpy.ok():
        rclpy.spin_once(getObjectRange_node)
    
    getObjectRange_node.destroy_node()
    rclpy.shutdown()