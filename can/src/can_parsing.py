#!/usr/bin/env python

import rospy
import cv2
import sys
import numpy as np
import copy
from mi_msgs.msg import *

class Can_parsing(object):
        ###################clock####################
    def __init__(self):
        self.velocity_current_time = 0
        self.velocity_pre_time = 0
        self.velocity_time_start = False
        self.velocity_dt = 0
        self.heading_current_time = 0
        self.heading_pre_time = 0
        self.heading_time_start = False
        self.heading_dt = 0
        self.sonar_1B0_current_time = 0
        self.sonar_1B0_pre_time = 0
        self.sonar_1B0_time_start = False
        self.sonar_1B0_dt = 0
        self.sonar_1C0_current_time = 0
        self.sonar_1C0_pre_time = 0
        self.sonar_1C0_time_start = False
        self.sonar_1C0_dt = 0
        self.sonar_rate_limit_value = 20 ###5cmpersec
        self.avm_301_current_time = 0
        self.avm_301_pre_time = 0
        self.avm_301_time_start = False
        self.avm_301_dt = 0
        self.avm_noise_criterion = 3 ###m
        #####################car_SCH_state######################
        self.car_info1 = {'accel_value':0,'brake_value':0,'gear_value':0,'steering_value':0,'switch_state':0}
        self.car_info2 = {'override_feedback':0,'vehicle_speed':0,'wheel_speed_rear_right':0,'wheel_speed_rear_left':0}
        self.car_info3 = {'lateral_acc':0,'long_acc':0,'yaw_rate':0}
        self.switch_name = ('AUTO','ASM','APM','AGM','MANUAL')
        self.gear_name = {0:'P',5:'D',6:'N',7:'R'}
        self.switch_base = [0,1,1,1,0,1] ###TODO
        #self.current_sonar_dict = {'S1':0,'S2':0,'S3':0,'S4':0,'L1':0,'L5':0,'L2':0,'L4':0,'L6':0,'L3':0,'S5':0,'S6':0,'S7':0,'S8':0}
        #self.pre_sonar_dict = {'S1':0,'S2':0,'S3':0,'S4':0,'L1':0,'L5':0,'L2':0,'L4':0,'L6':0,'L3':0,'S5':0,'S6':0,'S7':0,'S8':0}
        ################sonar###################
        self.sonar_name = ['S1','S2','S3','S4','L1','L5','L2','L4','L6','L3','S5','S6','S7','S8']
        self.current_sonar_1B0 = np.zeros(14,dtype=float)
        self.pre_sonar_1B0 = np.zeros(14,dtype=float)
        self.current_sonar_1C0 = np.zeros(14,dtype=float)
        self.pre_sonar_1C0 = np.zeros(14,dtype=float)
        self.car_SCH_state = Car()
        self.car_SCH_state.PosX = -0.91
        self.car_SCH_state.PosY = 2.43
        self.car_LGIT_state = Car_LGIT()
        self.pre_car_LGIT_state = Car_LGIT()
        self.sonar = Sonar()
        ########################IMU#########################
        self.imu_start = False
        self.imu_heading_init = 0
        self.imu_heading = 0
        self.pre_imu_heading = 0

    def initializer(self,signal):
        if signal:
            print("__init__update!")
            self.__init__()
    
    def location_update(self,signal):
        if signal:
            
            print("location_update!")
            if -220 < self.car_LGIT_state.heading < -150:
                self.car_SCH_state.PosX = self.car_LGIT_state.PosX + 0.50
                #self.car_SCH_state.PosX = self.car_LGIT_state.PosX
                #self.car_SCH_state.PosY = self.car_LGIT_state.PosY - 0.50
            
                self.car_SCH_state.left_right = -1
            else:
                self.car_SCH_state.left_right = 1
                self.car_SCH_state.PosX = self.car_LGIT_state.PosX
            
            self.car_SCH_state.PosX = self.car_SCH_state.PosX*self.car_SCH_state.left_right
            self.car_SCH_state.PosY = self.car_LGIT_state.PosY
            self.car_SCH_state.heading = (1-self.car_SCH_state.left_right)*90 + self.car_LGIT_state.heading
            ##########to test#########
            #self.car_SCH_state.PosX = 2.1
            #self.car_SCH_state.PosY = 2.1
            #self.car_SCH_state.heading = 0
            
            print("self.car_SCH_state.PosX",self.car_SCH_state.PosX)
            print("self.car_SCH_state.PosY",self.car_SCH_state.PosY)
    
    def imu_parser(self,receive_data):
        if not self.imu_start:
            self.imu_heading_init = receive_data.heading
            self.imu_start = True
            self.imu_heading = self.imu_heading_init - receive_data.heading
        if self.car_SCH_state.velocity == 0:
            self.imu_heading = self.pre_imu_heading
        else:
            self.imu_heading = self.imu_heading_init - receive_data.heading
        self.imu_heading = self.normalize_angle(self.imu_heading)
        self.pre_imu_heading = self.imu_heading
        #self.car_SCH_state.heading = self.imu_heading
        
    def car_SCH_parser_51(self,receive_data):
        self.parsing_51(receive_data['51'])
        #return self.car_SCH_state
    
    def car_SCH_parser_52(self,receive_data):
        self.parsing_52(receive_data['52'])
        #return self.car_SCH_state
    
    def car_SCH_parser_53(self,receive_data):
        self.parsing_53(receive_data['53'])
        #return self.car_SCH_state

    def car_LGIT_parser_300(self,receive_data):
        self.parsing_300(receive_data['300'])
        #return self.car_LGIT_state
    
    def car_LGIT_parser_301(self,receive_data):
        self.parsing_301(receive_data['301'])
        #return self.car_LGIT_state

    def car_LGIT_parser_302(self,receive_data):
        self.parsing_302(receive_data['302'])
        #return self.car_LGIT_state

    def sonar_parser_1B0(self,receive_data):
        self.parsing_1B0(receive_data['1B0'])
        #return self.sonar
   
    def sonar_parser_1C0(self,receive_data):
        self.parsing_1C0(receive_data['1C0'])
        #return self.sonar

    def SCH_car_pub(self):
        self.car_SCH_state.header.stamp = rospy.Time.now()
        self.car_SCH_state.header.frame_id = "can_sub"
        return self.car_SCH_state
    
    def LGIT_car_pub(self):
        return self.car_LGIT_state

    def sonar_pub(self):
        return self.sonar
    
    def parsing_51(self,data):
        self.car_info1['accel_value'] = self.merge_unsigned_16(data.data[0:2])
        self.car_info1['brake_value'] = self.merge_unsigned_16(data.data[2:4])
        self.car_info1['gear_value'] = data.data[4]
        self.car_info1['steering_value'] = int(self.merge_signed_16(data.data[5:7]) * 0.1)
        self.car_info1['switch_state'] = self.switch_state(data.data[7])
        self.car_SCH_state.steering = self.car_info1['steering_value']
        self.car_SCH_state.gear = self.car_info1['gear_value']
        
    def parsing_52(self,data):
        if self.velocity_time_start:
            self.velocity_current_time = data.header.stamp
            self.velocity_dt = (self.velocity_current_time.secs - self.velocity_pre_time.secs) + (self.velocity_current_time.nsecs - self.velocity_pre_time.nsecs)*0.000000001
            if self.car_SCH_state.gear == 7:
                velocity_ = self.car_SCH_state.velocity*-0.27777778
            else:
                velocity_ = self.car_SCH_state.velocity*0.27777778
            #self.distance += velocity_*self.velocity_dt
            #print("self.car_SCH_state.heading",self.car_SCH_state.heading)
            self.car_SCH_state.PosX += (velocity_)*np.cos(np.radians(self.car_SCH_state.heading))*self.velocity_dt   
            self.car_SCH_state.PosY -= (velocity_)*np.sin(np.radians(self.car_SCH_state.heading))*self.velocity_dt
            
        self.car_SCH_state.velocity_dt = self.velocity_dt
        self.velocity_pre_time = data.header.stamp   
        self.velocity_time_start = True
        self.car_info2['override_feedback'] = data.data[0]
        self.car_SCH_state.feedback = data.data[0]
        self.car_info2['vehicle_speed'] = self.merge_unsigned_16(data.data[1:3])*0.1
        self.car_info2['wheel_speed_rear_right'] = self.merge_unsigned_16(data.data[3:5])*0.1
        self.car_info2['wheel_speed_rear_left'] = self.merge_unsigned_16(data.data[5:7])*0.1
        self.car_SCH_state.velocity = (self.car_info2['wheel_speed_rear_right'] + self.car_info2['wheel_speed_rear_left']) * 0.5
    
    def parsing_53(self,data):
        if self.heading_time_start:
            self.heading_current_time = data.header.stamp
            self.heading_dt = (self.heading_current_time.secs - self.heading_pre_time.secs) + (self.heading_current_time.nsecs - self.heading_pre_time.nsecs)*0.000000001
            self.car_SCH_state.heading += self.car_info3['yaw_rate']*self.heading_dt*self.car_SCH_state.left_right
            #self.car_SCH_state.heading = self.normalize_angle(self.car_SCH_state.heading)
        self.heading_pre_time =  data.header.stamp
        self.heading_time_start = True
        self.car_info3['lateral_acc'] = (self.merge_unsigned_16(data.data[0:2])*0.01)-10.43
        self.car_info3['long_acc'] = (self.merge_unsigned_16(data.data[2:4])*0.01)-10.23
        self.car_info3['yaw_rate'] = (self.merge_unsigned_16(data.data[4:6])*0.01)-40.975
        self.car_info3['yaw_rate'] = round(self.car_info3['yaw_rate'],0)
        self.car_SCH_state.yaw_rate = self.car_info3['yaw_rate']
        self.car_SCH_state.yaw_rate_dt = self.heading_dt
        
    def parsing_1B0(self,data):
        data_bin = self.convert(data.data)
        if self.sonar_1B0_time_start:
            self.sonar_1B0_current_time = data.header.stamp
            self.sonar_1B0_dt = (self.sonar_1B0_current_time.secs - self.sonar_1B0_pre_time.secs) + (self.sonar_1B0_current_time.nsecs - self.sonar_1B0_pre_time.nsecs)*0.000000001
        self.sonar_1B0_pre_time =  data.header.stamp
        self.sonar_1B0_time_start = True
        self.current_sonar_1B0[self.sonar_name.index('L1')] = self.bin2oct(data_bin[16:23])*400/127
        self.current_sonar_1B0[self.sonar_name.index('S2')] = self.bin2oct(data_bin[23:30])*200/127
        self.current_sonar_1B0[self.sonar_name.index('S1')] = self.bin2oct(data_bin[33:40])*200/127
        self.current_sonar_1B0[self.sonar_name.index('L2')] = self.bin2oct(data_bin[40:47])*400/127
        self.current_sonar_1B0[self.sonar_name.index('S3')] = self.bin2oct(data_bin[47:54])*200/127
        self.current_sonar_1B0[self.sonar_name.index('S4')] = self.bin2oct(data_bin[54:61])*200/127
        #self.current_sonar_1B0 = copy.deepcopy(self.sonar_error_value_check(self.sonar_1B0_dt,self.current_sonar_1B0,self.pre_sonar_1B0))
        self.sonar.front[0] = self.current_sonar_1B0[self.sonar_name.index('S4')]
        self.sonar.front[1] = self.current_sonar_1B0[self.sonar_name.index('S3')]
        self.sonar.front[2] = self.current_sonar_1B0[self.sonar_name.index('S2')]
        self.sonar.front[3] = self.current_sonar_1B0[self.sonar_name.index('S1')]
        self.sonar.left[0] = self.current_sonar_1B0[self.sonar_name.index('L1')]
        self.sonar.left[1] = self.current_sonar_1B0[self.sonar_name.index('L2')]
        self.pre_sonar_1B0 = copy.deepcopy(self.current_sonar_1B0)
        
    def parsing_1C0(self,data): ###long range*400/127, short range *200/127
        data_bin = self.convert(data.data)
        if self.sonar_1C0_time_start:
            self.sonar_1C0_current_time = data.header.stamp
            self.sonar_1C0_dt = (self.sonar_1C0_current_time.secs - self.sonar_1C0_pre_time.secs) + (self.sonar_1C0_current_time.nsecs - self.sonar_1C0_pre_time.nsecs)*0.000000001
        self.sonar_1C0_pre_time = data.header.stamp
        self.sonar_1C0_time_start = True
        self.current_sonar_1C0[self.sonar_name.index('S6')] = self.bin2oct(data_bin[3:10])*200/127
        self.current_sonar_1C0[self.sonar_name.index('S7')] = self.bin2oct(data_bin[10:17])*200/127
        self.current_sonar_1C0[self.sonar_name.index('L4')] = self.bin2oct(data_bin[17:24])*400/127
        self.current_sonar_1C0[self.sonar_name.index('S5')] = self.bin2oct(data_bin[24:31])*200/127
        self.current_sonar_1C0[self.sonar_name.index('S8')] = self.bin2oct(data_bin[31:38])*200/127
        self.current_sonar_1C0[self.sonar_name.index('L3')] = self.bin2oct(data_bin[38:45])*400/127
        #self.current_sonar_1C0 = copy.deepcopy(self.sonar_error_value_check(self.sonar_1C0_dt,self.current_sonar_1C0,self.pre_sonar_1C0))
        self.sonar.rear[0] = self.current_sonar_1C0[self.sonar_name.index('S5')]
        self.sonar.rear[1] = self.current_sonar_1C0[self.sonar_name.index('S6')]
        self.sonar.rear[2] = self.current_sonar_1C0[self.sonar_name.index('S7')]
        self.sonar.rear[3] = self.current_sonar_1C0[self.sonar_name.index('S8')]
        self.sonar.right[0] = self.current_sonar_1C0[self.sonar_name.index('L4')]
        self.sonar.right[1] = self.current_sonar_1C0[self.sonar_name.index('L3')]
        self.pre_sonar_1C0 = copy.deepcopy(self.current_sonar_1C0)
        
    def parsing_300(self,data):
        self.car_LGIT_state.targetX1 = self.merge_signed_16(data.data[1:3])*0.1
        self.car_LGIT_state.targetX2 = self.merge_signed_16(data.data[3:5])*0.1
        
    def parsing_301(self,data): 
        self.car_LGIT_state.status = self.merge_unsigned_8(data.data[0])
        if self.avm_301_time_start:
            self.avm_301_current_time = data.header.stamp
            self.avm_301_dt = (self.avm_301_current_time.secs - self.avm_301_pre_time.secs) + (self.avm_301_current_time.nsecs - self.avm_301_pre_time.nsecs)*0.000000001
        self.avm_301_pre_time =  data.header.stamp
        self.avm_301_time_start = True
        self.car_LGIT_state.heading = self.merge_signed_16(data.data[5:7])*0.1 ##degree
        self.car_LGIT_state.heading = -1*(self.car_LGIT_state.heading + 90)
        #self.car_LGIT_state.heading = self.normalize_angle(self.car_LGIT_state.heading)
        self.car_LGIT_state.PosX = self.merge_signed_16(data.data[1:3])*0.1*-0.01 - 1.35*np.cos(np.radians(self.car_LGIT_state.heading)) ##cm 
        self.car_LGIT_state.PosY = self.merge_signed_16(data.data[3:5])*0.1*0.01 + 1.35*np.sin(np.radians(self.car_LGIT_state.heading)) ##cm
        self.pre_car_LGIT_state = copy.deepcopy(self.car_LGIT_state)
    
    def parsing_302(self,data): 
        self.car_LGIT_state.targetPosX1 = self.merge_signed_16(data.data[0:2])*0.1*0.01 ##cm 
        self.car_LGIT_state.targetPosY1 = self.merge_signed_16(data.data[2:4])*0.1*0.01 ##cm
        self.car_LGIT_state.targetPosX2 = self.merge_signed_16(data.data[4:6])*0.1*0.01 ##cm 
        self.car_LGIT_state.targetPosY2 = self.merge_signed_16(data.data[6:8])*0.1*0.01 ##cm
                            
    def bin2oct(self,data):
        bin_str = '0b' 
        for bit in data:
            bin_str += ('%s'%bit)
        return int(bin_str,2)

    def convert(self,data):
        data = map(bin,data)
        #print(data)
        room = np.zeros((8,8),dtype=np.uint8)
        for i,id in enumerate(data):
            swap = list(id.split('b')[-1])
            for index,bit in enumerate(swap):
                room[i,8-len(swap)+index] = bit
        room = room.reshape(64)
        #print(room)
        return room

    def merge_unsigned_8(self,data):
        return data

    def merge_unsigned_16(self,data):
        return np.uint16(((data[1]<<8)+data[0])&0xffff)
    
    def merge_signed_16(self,data):
        return np.int16(((data[1]<<8)+data[0])&0xffff)
    
    def sonar_error_value_check(self,dt,current_sonar,pre_sonar):
        current_sonar[np.where(abs(current_sonar - pre_sonar) > self.sonar_rate_limit_value)] = pre_sonar[np.where(abs(current_sonar - pre_sonar) > self.sonar_rate_limit_value)]
        return current_sonar
    
    def switch_state(self,data): ###TODO
        switch_bin = []
        for str_ in bin(data).split('b')[1]:
            switch_bin.append(int(str_))        
        switch_bin.reverse()
        return data

    def normalize_angle(self,angle):
        while angle > 180:
            angle -= 2.0 * 180
        while angle < -180:
            angle += 2.0 * 180
        return angle
