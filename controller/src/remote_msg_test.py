#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import array
import copy
import time
from mi_msgs.msg import *


class Controller(object):	
    def __init__(self):
        self.remote_pub = rospy.Publisher('remote_messages', Remote, queue_size = 10)
        self.final_cmd = Remote()
        self.count = 30
        self.final_cmd.velocity = 0 
        self.final_cmd.steering = 0
        self.final_cmd.gear = 0
        print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
        self.remote_pub.publish(self.final_cmd)
        
        self.remote_publish_2()
        
    def remote_publish(self):
        during_time = 120
        self.final_cmd.velocity = 0
        self.final_cmd.steering = 0
        self.final_cmd.gear = 5 
        
        while self.count != 0:
        #if count < 10
            print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
            self.remote_pub.publish(self.final_cmd)
            self.count -= 1
            #print(self.count)
            #time.sleep(1)
            #during_time = during_time - 1
        
        self.final_cmd.velocity = 0
        self.final_cmd.steering = 0
        self.final_cmd.gear = 0
        self.remote_pub.publish(self.final_cmd)

        '''
        self.final_cmd.velocity = 0
        self.final_cmd.steering = 0
        self.final_cmd.gear = 0
        
        cnt = 0
        while cnt < 2:
            print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
            self.remote_pub.publish(self.final_cmd)
        '''

    def remote_publish_2(self):
        during_time = 100
        self.final_cmd.velocity = 10
        self.final_cmd.steering = 100
        self.final_cmd.gear = 5
        
        while during_time != 0:
            print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
            self.remote_pub.publish(self.final_cmd)
            time.sleep(1)
            #during_time = during_time - 1
        '''
        self.final_cmd.velocity = 0
        self.final_cmd.steering = 0
        self.final_cmd.gear = 0
        
        cnt = 0
        while cnt < 2:
            print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
            self.remote_pub.publish(self.final_cmd)
        '''
        
			
if __name__ == '__main__':
	rospy.init_node('Remote',anonymous=True)
	controller = Controller()
	rospy.spin()
		
		
