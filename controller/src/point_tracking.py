#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import array
import copy
import time
import csv
import matplotlib.pyplot as plt
from mi_msgs.msg import *


class Algorithm_calc(object):
    def __init__(self):
        
        print("here init")
        
        self.L = 2.7 ###wheeeeeeeeel base ### [m] Wheel base of vehicle
        self.W = 1.72
        self.rtk_sub = rospy.Subscriber('/Pose_messages', RTK, self.rtk_callback)
        self.car_sub = rospy.Subscriber('/car_messages', Car, self.car_callback)

        self.remote_pub = rospy.Publisher('remote_messages', Remote, queue_size = 10)

        self.final_cmd = Remote()
        self.final_cmd.steering = 0

        ## path setting ##
        path_all_x = []
        path_all_y = []
        yaw_all = []

        path = './RTK_sujung_hu.csv'

        files = open(path, 'r')
        read = csv.reader(files)
        for f in read:
            path_all_x.append(float(f[0]))
            path_all_y.append(float(f[1]))
            yaw_all.append(float(f[2]))

        self.path_all_x = path_all_x
        self.path_all_y = path_all_y
        self.yaw_all = yaw_all

        self.current_index = 0
        self.current_point = 0
        self.now_point = 0


    def pure_pursuit(self):
        x = self.PosX
        y = self.PosY
        car_heading = float(self.heading)
        fx, fy, self.rear_x, self.rear_y = self.calc_wheels_utm(x, y, car_heading)

        if self.current_point == 0:
            dx_com_path = self.path_all_x[:300]
            dy_com_path = self.path_all_y[:300]
        else:
            dx_com_path = self.path_all_x[self.current_point: self.current_point + 150]
            dy_com_path = self.path_all_y[self.current_point: self.current_point + 150]

        dx = [self.rear_x - ix for ix in dx_com_path[:]]
        dy = [self.rear_y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        closest_error = min(d)
        current_index_yet = d.index(closest_error)

        ''' pure pursuit Algorithm '''
        Lf = (self.velocity * 0.237) + 2.9  #2.7 고주로 설정    #5.29

        print("previous self.current_point = %d"%(self.current_point))
        self.current_point = current_index_yet + self.current_point
        print("self.current_point = %d"%(self.current_point))

        find_check = False
        while Lf > d[current_index_yet]:
            #print("check point ")
            find_check = True
            if (current_index_yet + self.current_point + 1) >= len(self.path_all_x[:]) :
                find_check = False
                break
            else:
                current_index_yet = current_index_yet + 1
        if find_check:
            self.now_point = current_index_yet + self.current_point
        else:
            if self.now_point > 1170:
                self.now_point = current_index_yet + self.current_point + 2
            else:
                self.now_point = current_index_yet + self.current_point + 5

        self.current_index = self.now_point
        Lf_m = np.sqrt((self.path_all_x[self.now_point] - self.rear_x)**2 +
                       (self.path_all_y[self.now_point] - self.rear_y)**2)

        print("self.now_point = %d"%(self.now_point))
        print("self.yaw_all[self.now_point] = %f, car_heading = %f"%(self.yaw_all[self.now_point], car_heading))

        alpha = np.radians(self.yaw_all[self.now_point] - car_heading)
        print(">> alpha = %f"%(np.degrees(alpha)))
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf_m, 1.0)
        print("delta = %f"%(np.degrees(delta)))
        print("Lf_m = %f"%(Lf_m))
        
        abs_delta_degrees = np.degrees(delta)
              
        front_axle_vec = [np.cos(np.radians(car_heading)), -np.sin(np.radians(car_heading))]
        
        ##else: 직선일 때 이정도가 괜찮음
        #    steering_gain = -4.6
        #    stanely_steering_gain = -4.6
        steering_gain = -3.53  ## -3.3 이거 코너2 잘돔 조금 나감 조금 아래도  ### -5.0
        stanely_steering_gain = -3.53   ### -5.0

        #steering_gain = -5.65   ## 코너1 너무많이 돌음 이거     ### -5.0
        #stanely_steering_gain = -5.65   ### -5.0
        '''
        if self.now_point < (len(self.path_all_x[:])*1/2) and alpha > 7:
            print(">> here 1 <<")
            steering_gain = -5.5   ## 82였을 때 조금 더 많이 돎.    ### -5.0
            stanely_steering_gain = -5.5   ### -5.0
        elif self.now_point > 1170 and alpha > 7:
            print(">> here 2 <<")
            steering_gain = -2.2   ## 82였을 때 조금 더 많이 돎.    ### -5.0
            stanely_steering_gain = -2.2   ### -5.0   
        else:
            print(">> here 3 <<")
            steering_gain = -2.08  ## 원래는 3.88 아래도  ### -5.0
            stanely_steering_gain = -2.08   ### -5.0
        '''
        error_front_axle = np.dot([dx[current_index_yet], dy[current_index_yet]], front_axle_vec)
        k = 1.7 ## 직선일 때 1.7 괜찮음  # 2.0
        theta_d = np.arctan2(k*error_front_axle, (self.velocity + 3.0))
        print("error_front_axle = %f, theta_d = %f"%(error_front_axle, theta_d))

        steering = (np.degrees(delta) * steering_gain) - (np.degrees(theta_d) * stanely_steering_gain)#* 0.6
        print("steering value = ", steering)

        print("================================")

        return steering

    def calc_wheels_utm(self, now_utm_x, now_utm_y, now_heading, front_WheelBase_dist=-1.30, back_WheelBase_dist=1.45):
        
        now_heading = np.deg2rad(now_heading)
        front_WheelBase_utmX = (front_WheelBase_dist * np.cos(now_heading)) + now_utm_x
        front_WheelBase_utmY = (front_WheelBase_dist * np.sin(now_heading)) + now_utm_y

        #back_WheelBase_utmX = (back_WheelBase_dist * np.cos(now_heading)) + now_utm_x
        #back_WheelBase_utmY = (back_WheelBase_dist * np.sin(now_heading)) + now_utm_y
        back_WheelBase_utmX = now_utm_x
        back_WheelBase_utmY = now_utm_y
        return front_WheelBase_utmX, front_WheelBase_utmY, back_WheelBase_utmX, back_WheelBase_utmY

    def car_callback(self, car):
        self.velocity = car.velocity
        self.steering_real = car.steering
    
    def school_park(self):
        x = self.PosX
        y = self.PosY
        car_heading = float(self.heading)
        fx, fy, self.rear_x, self.rear_y = self.calc_wheels_utm(x, y, car_heading)

        if self.current_point == 0:
            dx_com_path = self.path_all_x[:300]
            dy_com_path = self.path_all_y[:300]
        else:
            dx_com_path = self.path_all_x[self.current_point: self.current_point + 150]
            dy_com_path = self.path_all_y[self.current_point: self.current_point + 150]

        dx = [self.rear_x - ix for ix in dx_com_path[:]]
        dy = [self.rear_y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        closest_error = min(d)
        current_index_yet = d.index(closest_error)

        ''' pure pursuit Algorithm '''
        Lf = (self.velocity * 0.237) + 2.9  #2.7 고주로 설정    #5.29

        print("previous self.current_point = %d"%(self.current_point))
        self.current_point = current_index_yet + self.current_point
        print("self.current_point = %d"%(self.current_point))

        find_check = False
        while Lf > d[current_index_yet]:
            #print("check point ")
            find_check = True
            if (current_index_yet + self.current_point + 1) >= len(self.path_all_x[:]) :
                find_check = False
                break
            else:
                current_index_yet = current_index_yet + 1
        if find_check:
            self.now_point = current_index_yet + self.current_point
        else:
            if self.now_point > 1170:
                self.now_point = current_index_yet + self.current_point + 2
            else:
                self.now_point = current_index_yet + self.current_point + 5

        self.current_index = self.now_point
        Lf_m = np.sqrt((self.path_all_x[self.now_point] - self.rear_x)**2 +
                       (self.path_all_y[self.now_point] - self.rear_y)**2)

        print("self.now_point = %d"%(self.now_point))
        print("self.yaw_all[self.now_point] = %f, car_heading = %f"%(self.yaw_all[self.now_point], car_heading))

        alpha = np.radians(self.yaw_all[self.now_point] - car_heading)
        print(">> alpha = %f"%(np.degrees(alpha)))
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf_m, 1.0)
        print("delta = %f"%(np.degrees(delta)))
        print("Lf_m = %f"%(Lf_m))
        
        abs_delta_degrees = np.degrees(delta)
              
        front_axle_vec = [np.cos(np.radians(car_heading)), -np.sin(np.radians(car_heading))]
        
        ##else: 직선일 때 이정도가 괜찮음
        #    steering_gain = -4.6
        #    stanely_steering_gain = -4.6
        steering_gain = -3.53  ## -3.3 이거 코너2 잘돔 조금 나감 조금 아래도  ### -5.0
        stanely_steering_gain = -3.53   ### -5.0

        #steering_gain = -5.65   ## 코너1 너무많이 돌음 이거     ### -5.0
        #stanely_steering_gain = -5.65   ### -5.0
        '''
        if self.now_point < (len(self.path_all_x[:])*1/2) and alpha > 7:
            print(">> here 1 <<")
            steering_gain = -5.5   ## 82였을 때 조금 더 많이 돎.    ### -5.0
            stanely_steering_gain = -5.5   ### -5.0
        elif self.now_point > 1170 and alpha > 7:
            print(">> here 2 <<")
            steering_gain = -2.2   ## 82였을 때 조금 더 많이 돎.    ### -5.0
            stanely_steering_gain = -2.2   ### -5.0   
        else:
            print(">> here 3 <<")
            steering_gain = -2.08  ## 원래는 3.88 아래도  ### -5.0
            stanely_steering_gain = -2.08   ### -5.0
        '''
        error_front_axle = np.dot([dx[current_index_yet], dy[current_index_yet]], front_axle_vec)
        k = 1.7 ## 직선일 때 1.7 괜찮음  # 2.0
        theta_d = np.arctan2(k*error_front_axle, (self.velocity + 3.0))
        print("error_front_axle = %f, theta_d = %f"%(error_front_axle, theta_d))

        steering = (np.degrees(delta) * steering_gain) - (np.degrees(theta_d) * stanely_steering_gain)#* 0.6
        print("steering value = ", steering)

        print("================================")

        return steering

    def rtk_callback(self, RTK):
        
        #print(">> here in")
        self.PosX = RTK.utm_x
        self.PosY = RTK.utm_y
        self.heading = RTK.heading
        idx = self.PosX - self.path_all_x[-3]
        idy = self.PosY - self.path_all_y[-3]
        
        d = np.sqrt(idx ** 2 + idy ** 2)
        if d < 30 and self.now_point > (len(self.path_all_x[:])*3/4):
            self.steering = 0     
        else:
            self.steering = self.pure_pursuit()     
        
        self.final_cmd.steering = self.steering
        self.remote_pub.publish(self.final_cmd)
        

        #with open('tlfwpwngod.csv', 'a') as f:
        #    f.write(str(self.PosX)+ ',' + str(self.PosY) + ',' + str(self.heading) + ',' + str(self.steering_real) + str("\n"))
        #    #f.write(str(self.PosX)+ ',' + str(self.PosY) + ',' + str(self.heading) +str("\n"))
        
        
			
if __name__ == '__main__':
	rospy.init_node('Remote',anonymous=True)
	controller = Algorithm_calc()
	rospy.spin()
		
		
