import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSReliabilityPolicy
import numpy as np
import math

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
        self.get_camera_msg=self.create_subscription(Int32MultiArray,'/object_x',self.camera_coordinate_callback,qos_profile)

        self.angle_pub = self.create_publisher(Float32, "/angle_measured", 10)
        self.dist_pub = self.create_publisher(Float32, "/dist_measured", 10)
        self.x_coordinate=np.zeros(2,dtype=float)

   
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
        #Preprocess the lidar data
        #self.get_logger().info(f"maximum angle: {msg.scan_time}")
        #self.get_logger().info(f"{self.x_coordinate}")
        
        self.Camera_Angle_Center_ROS_Msg = Float32()
        self.object_distance_ros_msg = Float32()
        
        if self.x_coordinate[0]!=800:

            Camera_Angle_Left=(((self.x_coordinate[0]-160)/320)*62.2)*np.pi/180
            Camera_Angle_Right=(((self.x_coordinate[0]+self.x_coordinate[1]-160)/320)*62.2)*np.pi/180
            Camera_Angle_Center=(((self.x_coordinate[0]+self.x_coordinate[1]*0.5-160)/320)*62.2)*np.pi/180
            
            self.Camera_Angle_Center_ROS_Msg.data = Camera_Angle_Center
            self.angle_pub.publish(self.Camera_Angle_Center_ROS_Msg)

            Left_index=int((Camera_Angle_Left-msg.angle_min)//msg.angle_increment)  #+180, for camera forward is 0, for lidar forward is pi 
            Right_index=int((Camera_Angle_Right-msg.angle_min)//msg.angle_increment)
            self.get_logger().info(f"This is Left Index {Left_index}") 
            self.get_logger().info(f"This is Right Index {Right_index}") 
        
            #angle_increment=1.478 degree
            if (Left_index < 0) & (Right_index >= 0):
                object_distance_dataset_ros=msg.ranges[Left_index:-1]
                object_distance_dataset_ros=np.append(object_distance_dataset_ros,msg.ranges[0:Right_index])
            else:
                object_distance_dataset_ros=msg.ranges[Left_index:Right_index+1]

            object_distance_dataset=np.array(object_distance_dataset_ros,dtype=float)
            

            object_distance_dataset_filtered=object_distance_dataset[((0.1<object_distance_dataset)&(object_distance_dataset<2))]
            self.get_logger().info(f"{object_distance_dataset_filtered}")

            if (np.mean(object_distance_dataset_filtered) != np.nan):
                self.object_distance_ros_msg.data = np.mean(object_distance_dataset_filtered)
            else:
                self.object_distance_ros_msg.data = float(0.0)
        
            self.dist_pub.publish(self.object_distance_ros_msg)

            # theta_sequence=np.arange(msg.angle_min,msg.angle_max,msg.angle_increment,dtype=float)[Left_index:Rigt_index+1]
            # mask=(0.1<object_distance_dataset)&(object_distance_dataset<2)
            # theta_sequence_mask=theta_sequence*mask
            # theta_sequence_selected=theta_sequence_mask[theta_sequence!=0]

            self.get_logger().info(f"This is the distance {np.mean(object_distance_dataset_filtered)}")
            
        else:
            self.Camera_Angle_Center_ROS_Msg.data = float(0.0)
            self.angle_pub.publish(self.Camera_Angle_Center_ROS_Msg)

            self.object_distance_ros_msg.data = float(0.0)
            self.dist_pub.publish(self.object_distance_ros_msg)
            self.get_logger().info("No object")

    def camera_coordinate_callback(self,msg:Int32MultiArray):
        self.x_coordinate=np.array(msg.data,dtype=float)
    
         
        
def main():
    rclpy.init()
    get_object_range_node = get_object_range()

    while rclpy.ok():
        rclpy.spin_once(get_object_range_node)

    get_object_range_node.destroy_node()
    rclpy.shutdown()