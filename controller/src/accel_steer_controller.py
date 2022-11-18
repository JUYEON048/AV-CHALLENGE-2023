#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String

class Accel_Steer_Controller(object):

    def __init__(self):
        self.Kp = 200.0 #200.0
        self.P = 0
        self.Ki = 0.01  #0.01
        self.Kc = self.Ki * 0.3  #0.3
        self.I = 0
        self.awu = 0
        #self.Kd = 0.1 #0.1
        #self.D = 0
        self.target_velocity = 0
        self.current_velocity = 0
        self.current_error = 0
        self.accel_cmd = 0

        self.jieun = rospy.Publisher('jieun_using', String, queue_size = 10)


    def p_control(self):  
        self.P = self.Kp * self.current_error
        return self.P
           
    def pi_control(self):
        self.I = self.I + (self.current_error*self.Ki - self.awu*self.Kc)
        self.accel_cmd = self.p_control() + self.I 
        self.awu = (self.p_control() + self.I) - self.accel_cmd
    
    def accel_control(self,target_velocity_,current_velocity_):
        #print('=' * 10)
        self.target_velocity = target_velocity_
        self.current_velocity = current_velocity_ 

        #if self.target_velocity <= (self.current_velocity + 4):
        #    self.current_velocity = self.target_velocity 
        
        #if self.current_velocity >= 20:
        #    self.current_velocity = self.current_velocity + 4
        #elif self.current_velocity >= 10:
        #    self.current_velocity = self.current_velocity + 2

        self.current_error = (self.target_velocity - self.current_velocity)
        self.pi_control()

        jieuns = "||         " + str(self.target_velocity) + "        ||        " + str(self.current_velocity) + "     ||      " + str(self.accel_cmd)
        self.jieun.publish(jieuns)
        #print("||\t", self.target_velocity, "\t||\t", self.current_velocity, "\t||\t", self.current_error)
        #print('*' * 10)
        return self.accel_cmd

    def rate_limiter(self,value,prev_value,max_change):
        delta = value - prev_value
        delta = np.clip(delta,-max_change,max_change)
        return prev_value + delta