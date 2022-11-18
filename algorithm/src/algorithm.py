#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import message_filters
from mi_msgs.msg import *
from algorithm_Highway import Highway
from algorithm_Hill import Hill
from algorithm_point_tracking import Point_tracking


class MI_Algorithms(object):
    def __init__(self):
        self.final_remote = Remote()
        self.point_tracking = Point_tracking()

        self.remote_pub = rospy.Publisher('remote_messages', Remote, queue_size = 10)
        obj_tracking_sub = message_filters.Subscriber("/Obj_tracking",Obj_tracking)
        car_sub = message_filters.Subscriber('car_messages', Car)
        obj_state_sub = message_filters.Subscriber("/Obj_state", Obj_state)
        obj_hill_state_sub = message_filters.Subscriber("/Obj_hill_state", Obj_hill_state)
        line_state_sub = message_filters.Subscriber("/line_state", Line)
        rtk_sub = message_filters.Subscriber("/Pose_messages", RTK)
        lane_lidar_sub = message_filters.Subscriber("/lane_state_lidar", Line)

        #[original]
        m_filters = message_filters.ApproximateTimeSynchronizer([obj_state_sub, line_state_sub, car_sub, rtk_sub, lane_lidar_sub], 10 , 100, allow_headerless=False)
        
        #[for test ACC]
        #m_filters = message_filters.ApproximateTimeSynchronizer([obj_state_sub, obj_hill_state_sub, line_state_sub, car_sub], 10 , 100, allow_headerless=False)
        
        #[for test Point tracking]
        #m_filters = message_filters.ApproximateTimeSynchronizer([car_sub, rtk_sub], 10 , 100, allow_headerless=True)

        #[for test LiDAR Lane tracking]
        #m_filters = message_filters.ApproximateTimeSynchronizer([line_state_sub, lane_lidar_sub], 10 , 100, allow_headerless=False)        

        m_filters2 = message_filters.ApproximateTimeSynchronizer([obj_state_sub, obj_hill_state_sub, car_sub, obj_tracking_sub], 10 , 100, allow_headerless=False)
        m_filters.registerCallback(self.Algorithms_normal)
        m_filters2.registerCallback(self.Algorithms_hill)
        
        #CONFIG velocity
        #self.final_remote.velocity = 40 
        self.final_remote.velocity = 20
        self.final_remote.steering = 0 
        self.final_remote.gear = 5 

        #CONFIG DRIVE MODE
        self.Highway_Flag = False
        self.Hill_Flag = True

    
    #def Algorithms_normal(self, obj_state, obj_hill_state_sub, line_state, RTK, lane_lidar):
    def Algorithms_normal(self, obj_state, line_state_sub, car_sub, RTK, lane_lidar_sub):
    #def Algorithms_normal(self, car_msgs,  RTK):
        print("--"*40)
        print("MI_Algorithms_N :: %s" %(rospy.Time.now()))
        print("velocity :: ", self.final_remote.velocity)
        
        # ========= speed calculation =========
        before_velocity = self.final_remote.velocity
        if (before_velocity > 10) and (before_velocity < 25):
            before_dist = 1
        else:
            before_dist = 0

        if self.Highway_Flag:   # Highway(ACC & AEB) algorithm
            if obj_state.data < 1000:
                self.final_remote.velocity = Highway(obj_state.data, before_dist, before_velocity)
                if self.final_remote.velocity == 0: #Emergency
                    while True:
                        self.remote_pub.publish(self.final_remote)
                        time.sleep(0.02)
                        break
                else:
                    pass
            else:
                self.final_remote.velocity = 40
            before_velocity = self.final_remote.velocity
            #print("gear :: ", self.final_remote.gear)
    

            # ========= steer calculation =========
            #from CAMERA
            '''if line_state is not None: 
                #self.final_remote.steering = line_state.data
                steering_fromCAMERA = line_state.data'''

            #from RTK
            if RTK.heading is not None:
                steering_fromPointTracking = self.point_tracking.pure_pursuit(RTK.utm_x, RTK.utm_y, RTK.heading, car_msgs.velocity)

            #from LiDAR
            '''if lane_lidar is not None: 
                #self.final_remote.steering = line_state.data
                steering_fromLiDAR = lane_lidar.data'''


            #self.final_remote.steer = steering_fromPointTracking #not yet
            self.final_remote.steering = 8
            self.remote_pub.publish(self.final_remote)
        else:
            pass
            


    def Algorithms_hill(self, obj_state, obj_hill_state, car_state, steer):
        print("--"*40)
        print("MI_Algorithms_H :: %s" %(rospy.Time.now()))
        print("velocity :: ", self.final_remote.velocity)
        if self.Hill_Flag:   # Hill(ACC & AEB) algorithm
            self.final_remote.steering = steer.data
            if obj_state.data < 70:
                self.final_remote.velocity = Hill(obj_hill_state.data)
                if car_state.velocity == 0 and self.final_remote.velocity == 0:
                    self.final_remote.gear = 6 #Neutral Gear
                    #self.final_remote.gear = 0 #Parking Gear
                else:
                    self.final_remote.gear = 5 #Drive Gear
                if self.final_remote.velocity == 0: #Emergency
                    while True:
                        self.remote_pub.publish(self.final_remote)
                        time.sleep(0.02)            
                        break
                else:
                    pass
            else:
                self.final_remote.velocity = 20
                self.final_remote.gear = 5
            self.remote_pub.publish(self.final_remote)
            
            #print("gear :: ", self.final_remote.gear)
        else:
            pass



if __name__ == '__main__':
    rospy.init_node('algorithm',anonymous=True)
    mi_algo = MI_Algorithms()
    rospy.spin()
