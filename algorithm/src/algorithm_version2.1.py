#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import message_filters
from mi_msgs.msg import *
from algorithm_Highway import Highway
from algorithm_Hill import Hill


class MI_Algorithms(object):
    def __init__(self):
        self.final_remote = Remote()
        self.remote_pub = rospy.Publisher('remote_messages', Remote, queue_size = 10)
        obj_tracking_sub = message_filters.Subscriber("/Obj_tracking", Obj_tracking)
        car_sub = message_filters.Subscriber('car_messages', Car)
        obj_state_sub = message_filters.Subscriber("/Obj_state", Obj_state)
        line_state_sub = message_filters.Subscriber("/line_state", Line)

        m_filters = message_filters.ApproximateTimeSynchronizer([obj_state_sub, line_state_sub], 10 , 100, allow_headerless=False)
        m_filters2 = message_filters.ApproximateTimeSynchronizer([obj_state_sub, car_sub, obj_tracking_sub], 10 , 100, allow_headerless=False)
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

    
    def Algorithms_normal(self, obj_state, line_state):
        #print("--"*40)
        print("MI_Algorithms_N :: %s" %(rospy.Time.now()))
        #print("velocity :: ", self.final_remote.velocity)
        
        # --------- speed calculation -----------
        if before_velocity > 10 and before_velocity < 25:
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
    
        # --------- steer calculation -----------
        else:
            if line_state is not None:
                self.final_remote.steering = line_state.data
            else:
                pass
        self.remote_pub.publish(self.final_remote)
        before_velocity = self.final_remote.velocity



    def Algorithms_hill(self, obj_state, car_state, steer):
        #print("--"*40)
        print("MI_Algorithms_H :: %s" %(rospy.Time.now()))
        #print("velocity :: ", self.final_remote.velocity)
        if self.Hill_Flag:   # Hill(ACC & AEB) algorithm
            self.final_remote.steering = steer.data
            if obj_state.data < 70:
                self.final_remote.velocity = Hill(obj_state.data)
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
