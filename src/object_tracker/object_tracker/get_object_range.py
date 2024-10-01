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
        x=distance*math.cos(angle)
        y=distance*math.sin(angle)
        
        
        #Gain the point that has maximum distance to the line
        segment_num=1
        division_index=np.array([0],dtype=int)
        collected_k=np.empty((0),dtype=float)
        collected_b=np.empty((0),dtype=float)
        init_flag=False

        while True:
            if init_flag==False:
                init_flag==True
                A = np.vstack([x, np.ones(len(x))]).T
                #Linear regression, return slope k and intersection b
                #Line equation: y=kx+b->kx-y+b=0
                #distance abs(kx-y+b)/sqrt(k^2+(-1)^2)
                k, b = np.linalg.lstsq(A, y, rcond=None)
                point_dis_test=math.fabs(k*x-y+b)/math.sqrt(k*k+1)
                if np.max(point_dis_test)<0.010:
                    break
                else:
                    division_index=np.append(division_index,np.argmax(point_dis_test,axis=0)[0])
                    segment_num+=1
            else:
                new_segment_flag=False
                for i in range(segment_num): #Perform the distance test for each segment
                    #segmented coordinates for i-th line
                    x_segment=x[division_index[i]:division_index[i+1]]
                    y_segment=y[division_index[i]:division_index[i+1]]            
                    A = np.vstack([x_segment, np.ones(len(x_segment))]).T
                    #Linear regression, return slope k and intersection b
                    # Line equation: y=kx+b->kx-y+b=0
                    #distance abs(kx-y+b)/sqrt(k^2+(-1)^2)
                    k, b = np.linalg.lstsq(A, y_segment, rcond=None)
                    point_dis_test=math.fabs(k*x_segment-y_segment+b)/math.sqrt(k*k+1)
                    if np.max(point_dis_test)<0.010:
                        collected_k=np.append(collected_k,k)
                        collected_b=np.append(collected_b,b)
                        continue
                    else:
                        division_index=np.append(division_index,np.argmax(point_dis_test,axis=0)[0])
                        segment_num+=1
                        new_segment_flag=True
                if new_segment_flag==False:
                    break
        return division_index,collected_k,collected_b
        


    #@Argument:
    # self: Pass the created instance itself, so that the defined method here can have access to other method in the class 
    # msg:Pose msg here is a name of variance, and :Pose indicate the type of variable
    
    def Lidar_Scan_callback(self,msg:LaserScan): 
        #Preprocess the lidar data
        #self.get_logger().info(f"maximum angle: {msg.scan_time}")
        #self.get_logger().info(f"{self.x_coordinate}")
        self.get_logger().info(f"{msg.angle_min}")
        self.get_logger().info(f"{msg.angle_max}")
        
        if self.x_coordinate!=800:

            Camera_Angle_Left=(((self.x_coordinate[0]-160)/320)*62.2)*np.pi/180
            Camera_Angle_Right=(((self.x_coordinate[0]+self.x_coordinate[1]-160)/320)*62.2)*np.pi/180
            Camera_Angle_Center=(((self.x_coordinate[0]+self.x_coordinate[1]*0.5-160)/320)*62.2)*np.pi/180
            self.angle_pub.publish(Camera_Angle_Center)

            Left_index=(Camera_Angle_Left+180-msg.angle_min)//msg.angle_increment  #+180, for camera forward is 0, for lidar forward is pi 
            Rigt_index=(Camera_Angle_Right+180-msg.angle_min)//msg.angle_increment

            #angle_increment=1.478 degree
            
            theta_sequence=np.arange(msg.angle_min,msg.angle_max,msg.angle_increment,dtype=float)[Left_index,Rigt_index+1]
            mask=(0.1<object_distance_dataset & object_distance_dataset<2)
            theta_sequence_selected=theta_sequence*mask
            theta_sequence_selected=theta_sequence_selected[theta_sequence!=0]

            object_distance_dataset=msg.ranges[Left_index:Rigt_index+1]
            object_distance_dataset_filtered=object_distance_dataset[(0.1<object_distance_dataset & object_distance_dataset<2)]

    def camera_coordinate_callback(self,msg:Int32MultiArray):
         self.x_coordinate=float(msg.data)
    
         

    

 
         

        

def main(args=None):
        rclpy.init(args=args)
        node=get_object_range() #create the node
        rclpy.spin(node) #hold the node running
        rclpy.shutdown()
       