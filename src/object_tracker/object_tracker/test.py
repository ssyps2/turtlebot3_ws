import math
import numpy as np

dataset=np.array([[ 1.4269781 ,  1.57079633],
 [ 0.827239  ,  1.44838575],
 [ 2.50300621,  1.48999741],
 [ 2.01310235,  1.41969297],
 [ 2.19045526,  1.38527902],
 [ 2.07403962,  1.32481262],
 [ 2.59302042,  1.33488655],
 [ 2.12316993,  1.23128542],
 [ 2.7573757 ,  1.2733689 ],
 [ 2.60701167,  1.21460203],
 [ 2.44872873,  1.14559942],
 [ 3.2520345 ,  1.22210683],
 [ 3.13437608,  1.17372575],
 [ 3.58222681,  1.19547829],
 [ 2.88972379,  1.05943021],
 [ 4.61426703,  1.23622707],
 [ 4.48240134,  1.20193123],
 [ 4.49865598,  1.17915312],
 [ 4.74074824,  1.17718928],
 [ 5.17179427,  1.19061569],
 [ 5.05820609,  1.15994385],
 [ 6.02455094,  1.21098821],
 [ 5.2629695 ,  1.13488428],
 [ 6.33772296,  1.19547438],
 [ 7.68657625,  1.24993241],
 [ 6.61878514,  1.17934744],
 [ 6.95343941,  1.18349328],
 [ 6.85440197,  1.16158455],
 [ 7.37970536,  1.17748234],
 [ 8.0450043 ,  1.19811577],
 [ 9.00921779,  1.22775174],
 [ 7.50162213,  1.14019409],
 [ 8.18864079,  1.16501956],
 [ 8.87209879,  1.1856339 ],
 [ 7.9106025 ,  1.12170809],
 [ 8.43405504,  1.13825877],
 [ 7.82595058,  1.08755186],
 [ 6.56266563,  0.96491081],
 [ 6.80580939,  0.97159105],
 [ 7.10919791,  0.98348323],
 [ 8.23757866,  1.05815078],
 [ 6.71187729,  0.90583628],
 [ 6.42726125,  0.84988813],
 [ 7.52646106,  0.95563715],
 [ 6.95353324,  0.87738627],
 [ 7.35955237,  0.90507513],
 [ 7.05688299,  0.85206524],
 [ 6.92836187,  0.81588613],
 [ 6.83922228,  0.78282574],
 [ 6.56148012,  0.71616949],
 [ 6.40604335,  0.66259757],
 [ 6.52772892,  0.6613331 ],
 [ 7.06984868,  0.73333338],
 [ 7.01458047,  0.70254361],
 [ 6.810899  ,  0.64207403],
 [ 7.24217408,  0.69646989],
 [ 6.48428404,  0.51080739],
 [ 6.92264071,  0.58862978],
 [ 6.49578025,  0.44663326],
 [ 6.75135163,  0.48916325],
 [ 6.96875061,  0.51623596],
 [ 7.14348232,  0.53050678],
 [ 7.04098232,  0.47464836],
 [ 7.15271551,  0.47414991],
 [ 7.41636416,  0.51219006],
 [ 7.07148804,  0.38052741],
 [ 6.956034  ,  0.28945166],
 [ 6.87835486,  0.17963361],
 [ 7.12771607,  0.27041965],
 [ 7.05796596,  0.15831897],
 [ 9.0039012 ,  0.66762593],
 [ 9.10902384,  0.66434619],
 [ 9.24414867,  0.66528894],
 [ 9.72210455,  0.70986366],
 [ 9.23345872,  0.62744788],
 [ 9.82415698,  0.69017338],
 [10.06777936,  0.70361471],
 [ 9.5665728 ,  0.62148262],
 [ 9.83899838,  0.64221444],
 [10.62392158,  0.72104533],
 [ 9.93442859,  0.62079809],
 [ 9.81494396,  0.58518803],
 [10.15472107,  0.61692238],
 [10.44361373,  0.63886714],
 [10.47483101,  0.62660589],
 [10.23570822,  0.57569467],
 [10.78489869,  0.63433505],
 [10.83726079,  0.6251167 ],
 [10.73267934,  0.59489417],
 [11.17830637,  0.63642217],
 [11.18134111,  0.62143674],
 [11.5396369 ,  0.6492262 ],
 [11.23057829,  0.59621534],
 [10.98425818,  0.54482429],
 [11.93637523,  0.65102446],
 [11.73697874,  0.61359468],
 [11.8603796 ,  0.61357717],
 [12.49383138,  0.6693496 ],
 [12.26807865,  0.63192824],
 [12.52787971,  0.64646248]]


)



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

distance=dataset[:,0]
angle=dataset[:,1]
division_index,collected_k,collected_b=Lidar_Point_Segment(distance,angle)
print(division_index)
print(collected_k)
print(collected_b)
