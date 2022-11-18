#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Int32
from mi_msgs.msg import *
from accel_steer_controller import *



class Controller(object):	
    def __init__(self):
        self.remote_pub = rospy.Publisher('remote_messages', Remote, queue_size = 10)
        self.final_cmd = Remote()
        self.ESOPT_FLAG = False
        self.final_cmd.velocity = 0 
        self.final_cmd.steering = 0
        self.final_cmd.gear = 0
        #print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
        self.remote_pub.publish(self.final_cmd)
        self.etop_flag_sub = rospy.Subscriber('Estop_flag',Int32,self.Estop_flag_check)

    def Estop_flag_check(self, flag):
        self.Estop_flag = flag
        #print("Etop_flag msg >>", self.Estop_flag.data)

        if self.Estop_flag.data == 1:
            self.final_cmd.velocity = 0 
            self.final_cmd.steering = 0
            self.final_cmd.gear = 0
            self.remote_pub.publish(self.final_cmd)
            #print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
            self.ESOPT_FLAG = True

        else :
            if self.ESOPT_FLAG is True:
                self.final_cmd.velocity = 0
                self.final_cmd.steering = 0
                self.final_cmd.gear = 0
                self.remote_pub.publish(self.final_cmd)
                #print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
            else:
                self.final_cmd.velocity = 40
                self.final_cmd.steering = 0
                self.final_cmd.gear = 5 
                #print("remote msg >>", self.final_cmd.velocity, self.final_cmd.steering, self.final_cmd.gear)
                self.remote_pub.publish(self.final_cmd)
                time.sleep(1)
        


			
if __name__ == '__main__':
	rospy.init_node('Remote',anonymous=True)
	controller = Controller()
	rospy.spin()
		
		
