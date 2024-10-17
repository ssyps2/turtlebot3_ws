import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSReliabilityPolicy
import numpy as np


#Datatype package
#It import a library with corresponding datatype. The datatype can be determined by ros2 topic info /nodename

class getObjectRange(Node):
    def __init__(self):
        #format: .fcn() or .instance
        super().__init__("getObjectRange")
        self.get_logger().info("getObjectRange Started")
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.get_Lidar_msg = self.create_subscription(LaserScan,'/scan', self.Lidar_Scan_callback, qos_profile) #return a subscribe instance
        self.vector_pub = self.create_publisher(Float64MultiArray, '/obstacle_vector',10)

    def Lidar_Point_Segment(distance:np.ndarray,angle:np.ndarray):
        # @para: 
        # distance: Lidar distance array
        # angle: Lidar angular position array
        # return: the index of segmented points in x and y coordinates

        regression_threshold=1

        x=distance*np.cos(angle)
        y=distance*np.sin(angle)
        
        
        #Gain the point that has maximum distance to the line
        segment_num=1
        division_index=np.array([[0],[len(x)-1]],dtype=int)
        collected_k=np.empty((0),dtype=float)
        collected_b=np.empty((0),dtype=float)
        init_flag=False

        while True:
            if init_flag==False:
                init_flag=True
                A = np.vstack([x, np.ones(len(x))]).T
                #Linear regression, return slope k and intersection b
                #Line equation: y=kx+b->kx-y+b=0
                #distance abs(kx-y+b)/sqrt(k^2+(-1)^2)
                k, b = np.linalg.lstsq(A, y, rcond=None)[0]
                point_dis_test=np.fabs(k*x-y+b)/np.sqrt(k*k+1)
                if np.max(point_dis_test)<regression_threshold:
                    collected_k=np.append(collected_k,k)
                    collected_b=np.append(collected_b,b)
                    break
                else:
                    division_index=np.insert(division_index,-1,np.argmax(point_dis_test[1:-1],axis=0)) #remove the start and end points
                    segment_num+=1
            else:
                new_segment_flag=False
                print(f"segment_num={segment_num}")
                print(division_index)

                division_index_instance=division_index #Memory the index before going into the for-loop
                for i in range(segment_num): #Perform the distance test for each segment
                    #segmented coordinates for i-th line
                    
                    x_segment=x[division_index_instance[i]:division_index_instance[i+1]]
                    y_segment=y[division_index_instance[i]:division_index_instance[i+1]]           
                    A = np.vstack([x_segment, np.ones(len(x_segment))]).T
                    #Linear regression, return slope k and intersection b
                    # Line equation: y=kx+b->kx-y+b=0
                    #distance abs(kx-y+b)/sqrt(k^2+(-1)^2)
                    k, b = np.linalg.lstsq(A, y_segment, rcond=None)[0]
                    point_dis_test=np.fabs(k*x_segment-y_segment+b)/np.sqrt(k*k+1)
                    print(f"hi {i} time")
                    
                    insert_locate=int(np.argwhere(division_index==division_index_instance[i]))+1 #insert the new index refering to the previous index
                    print(insert_locate)
                    print(division_index)
                    
                    if np.size(point_dis_test[2:-2])>0:
                        if np.max(point_dis_test[2:-2])<regression_threshold:
                            collected_k=np.append(collected_k,k)
                            collected_b=np.append(collected_b,b)
                            print(division_index)
                            print("pass")
                            continue
                        else:
                            print("add")
                            insert_value=(np.argmax(point_dis_test[2:-2])+division_index_instance[i])
                            if insert_value==division_index_instance[i]: # Avoid add same index, when agrmax=[0]
                                continue
                            division_index=np.insert(division_index,insert_locate,insert_value,axis=0)
                            print(division_index)
                            segment_num+=1
                            new_segment_flag=True
                if new_segment_flag==False:
                    break   
            # print(division_index)
            # for i in range(segment_num):
            #     print(f"{i}-th segment line is y={collected_k[i]:.2g}x+{collected_b[i]:.2g}")
        return division_index,collected_k,collected_b



    #@Argument:
    # self: Pass the created instance itself, so that the defined method here can have access to other method in the class 
    # msg:Pose msg here is a name of variance, and :Pose indicate the type of variable
    
    def Lidar_Scan_callback(self,msg:LaserScan): 

        #### Step1, Abstract valued points measured from Lidar ###
        detect_radius = 1.5
        detect_FoV = 120
        
        detect_radius_range_right = np.arange(
            msg.angle_min, detect_FoV * 0.5, msg.angle_increment, dtype=float
            ).flatten
        detect_radius_range_left = np.arange(
            msg.angle_max - detect_FoV * 0.5, msg.angle_max, msg.angle_increment, dtype=float
            ).flatten
        detect_radius_sequence = np.append(detect_radius_range_right, detect_radius_range_left) # radius sequence for detected region
        

        range_data_size = np.shape(msg.ranges)
        ranges_index_right = np.arange(0, np.shape(detect_radius_range_right)[0] ,1).flatten
        ranges_index_left = np.arange(
            range_data_size - np.shape(detect_radius_range_left), range_data_size, 1
            ).flatten
        ranges_index=np.append(ranges_index_right, ranges_index_left)
        Lidar_ranges = msg.ranges[ranges_index] # Abstract points within the

        # Abstracted range sequence of Lidar points according to wanted FoV and detection radius
        Lidar_ranges = Lidar_ranges[Lidar_ranges > detect_radius]
        # Abstracted radius sequence of Lidar points according to wanted FoV and detection radius
        detect_radius_sequence = detect_radius_sequence[Lidar_ranges > detect_radius]

        #### Step2, publish the vector to the nearest points ###
        nearest_index = np.argmin(Lidar_ranges)
        vector = Float64MultiArray()
        vector = [Lidar_ranges[nearest_index], detect_radius_sequence[nearest_index]]
        self.vector_pub.publish(vector)
        



       
        
         
        
def main():
    rclpy.init()
    get_object_range_node = getObjectRange()

    while rclpy.ok():
        rclpy.spin_once(get_object_range_node)

    get_object_range_node.destroy_node()
    rclpy.shutdown()