#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import csv
from std_msgs.msg import Int32

class Point_tracking(object):
    def __init__(self):
        self.L = 2.7 #[m] Wheel base of vehicle
        #self.current_point = 0
        self.now_point = 0 
        self.line_change_sub = rospy.Subscriber("line_change", Int32, self.line_change_callback)
        ## path setting ##
        path_all_x = []
        path_all_y = []
        yaw_all = []
        ## High ##
        file_path = '/home/mi/03HDW_env.3.1/src/algorithm/src' + '/tlfwpwngod.csv'
        #self.current_point = 600
        self.riding_state = 0
        ## deungpanlo ## 
        #file_path = '/home/mi/03HDW_env.3.1/src/algorithm/src' + '/emdvksfh_ehdwpdhQkrk.csv'
        #self.riding_state = 1
        ## 처음시작 인덱스
        #self.current_point = 4
        ## 첫번째 코너 나와서
        #self.current_point = 600
        ## 두번째 코너 전
        #self.current_point = 1600
        ## 두번째 코너 나와서
        self.current_point = 2400
        #file_path = '/home/mi/03HDW_env.3.1/src/algorithm/src' + '/3cktjs.csv'
        #self.current_point = 10
        with open(file_path, 'r') as files:
            read = csv.reader(files)
            for f in read:
                path_all_x.append(float(f[0]))
                path_all_y.append(float(f[1]))
                yaw_all.append(float(f[2]))
            self.path_all_x = path_all_x
            self.path_all_y = path_all_y
            self.yaw_all = yaw_all

        ## for line 2
        self.path_all_x2 = []
        self.path_all_y2 = []
        self.yaw_all2 = []

        path2 = '/home/mi/03HDW_env.3.1/src/algorithm/src' + '/2cktjs_ektl.csv'

        files2 = open(path2, 'r')
        read2 = csv.reader(files2)
        for f2 in read2:
            self.path_all_x2.append(float(f2[0]))
            self.path_all_y2.append(float(f2[1]))
            self.yaw_all2.append(float(f2[2]))
            
        self.current_point2 = 10
        self.now_point2 = 50
        self.line_chage_state = 0


    def pure_pursuit(self, utm_x, utm_y, heading, now_velocity):
        idx = utm_x - self.path_all_x[-3]
        idy = utm_y - self.path_all_y[-3]

        dist = np.sqrt(idx ** 2 + idy ** 2)
        if dist < 30 and self.now_point > (len(self.path_all_x[:])*3/4):
            steering = 0     
        else:
            #steering = self.pure_pursuit_calc(utm_x, utm_y, heading, now_velocity)
            #steering = self.pure_pursuit_for_line2(utm_x, utm_y, heading, now_velocity)
            
            if self.line_chage_state == 0:
                steering = self.pure_pursuit_calc(utm_x, utm_y, heading, now_velocity)
            #elif self.line_chage_state == 1:
            #    steering = 3.0
            #    print("111111111111111111")
            #    self.line_chage_state = 3
            elif self.line_chage_state == 1:
                print("22222222222222222222222")
                steering = self.pure_pursuit_for_line2(utm_x, utm_y, heading, now_velocity)
            #elif self.line_chage_state == 2:
            #    print("333333333333333333333")
            #    steering = -3.0
            #    self.line_chage_state =4 
            elif self.line_chage_state == 2:
                print("4444444444444444444444444")
                steering = self.pure_pursuit_calc(utm_x, utm_y, heading, now_velocity)
                self.line_chage_state = 0
            
        return steering

    def line_change_callback(self, data):
        self.line_chage_state = data.data

    def pure_pursuit_calc(self, utm_x, utm_y, heading, now_velocity):
        car_heading = float(heading)
        self.rear_x, self.rear_y = self.calc_wheels_utm(utm_x, utm_y, car_heading)
        
        if self.current_point == 0:
            dx_com_path = self.path_all_x[:]
            dy_com_path = self.path_all_y[:]
        #elif self.line_chage_state == 4:
        #    dx_com_path = self.path_all_x[self.current_point: self.current_point + 800]
        #    dy_com_path = self.path_all_y[self.current_point: self.current_point + 800]
        else:
            dx_com_path = self.path_all_x[self.current_point: self.current_point + 800]
            dy_com_path = self.path_all_y[self.current_point: self.current_point + 800]
        
        dx = [self.rear_x - ix for ix in dx_com_path[:]]
        dy = [self.rear_y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        
        closest_error = min(d)
        current_index_yet = d.index(closest_error)

        ''' pure pursuit Algorithm '''
        Lf = (now_velocity * 0.237) + 2.7    #5.29 곡선할 때
        #Lf = (now_velocity * 0.237) + 2.79    ##차선 변경ㅇ용

        self.current_point = current_index_yet + self.current_point

        find_check = False
        while Lf > d[current_index_yet]:
            find_check = True
            if (current_index_yet + self.current_point + 1) >= len(self.path_all_x[:]) :
                find_check = False
                break
            else:
                current_index_yet = current_index_yet + 1
        print("Flag 1 >> now_point = ", self.current_point)
        if find_check:
            self.now_point = current_index_yet + self.current_point + 3
        else:
            self.now_point = self.current_point + 5
        print("Flag 1 >> now_point = ", self.now_point)

        Lf_m = np.sqrt((self.path_all_x[self.now_point] - self.rear_x)**2 +
                       (self.path_all_y[self.now_point] - self.rear_y)**2)
        
        alpha = np.radians(self.yaw_all[self.now_point] - car_heading)
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf_m, 1.0)
        if self.riding_state == 0:
            steering_gain = -5.38
            stanely_steering_gain = -6.0
        elif self.riding_state == 1:
            steering_gain = -3.88
            stanely_steering_gain = -3.88
        ### 차선 변경용
        if self.now_point < 1600:
            steering_gain = -2.68
            stanely_steering_gain = -3.33

        front_axle_vec = [np.cos(np.radians(car_heading)), -np.sin(np.radians(car_heading))]
        error_front_axle = np.dot([dx[current_index_yet], dy[current_index_yet]], front_axle_vec)
        k = 1.7
        theta_d = np.arctan2(k*error_front_axle, (now_velocity + 3.0))
        steering = (np.degrees(delta) * steering_gain) - (np.degrees(theta_d) * stanely_steering_gain)#* 0.6
        #print("now_point ", self.now_point)
        if self.now_point < 5:
            steering = 3
        
        return steering

    def pure_pursuit_for_line2(self, utm_x, utm_y, heading, now_velocity):
        car_heading = float(heading)
        rear_x, rear_y = self.calc_wheels_utm(utm_x, utm_y, car_heading)

        dx_com_path = self.path_all_x2[self.current_point2: ]
        dy_com_path = self.path_all_y2[self.current_point2: ]
        
        dx = [rear_x - ix for ix in dx_com_path[:]]
        dy = [rear_y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        closest_error = min(d)
        current_index_yet = d.index(closest_error)

        ''' pure pursuit Algorithm '''
        Lf = (now_velocity * 0.237) + 2.78   #5.29

        self.current_point2 = current_index_yet + self.current_point2

        find_check = False
        while Lf > d[current_index_yet]:
            find_check = True
            if (current_index_yet + self.current_point2 + 2) >= len(self.path_all_x2[:]) :
                find_check = False
                break
            else:
                current_index_yet = current_index_yet + 1
        if current_index_yet > 300:
            now_point = self.current_point2 + 15
        elif find_check:
            now_point = current_index_yet + self.current_point2 + 1
        else:
            now_point = current_index_yet + self.current_point2 + 5
        if now_point > len(self.path_all_x2[:]):
            now_point = 900
        print("==========now_point = ", now_point)
        Lf_m = np.sqrt((self.path_all_x2[now_point] - rear_x)**2 + (self.path_all_y2[now_point] - rear_y)**2)

        alpha = np.radians(self.yaw_all2[now_point] - car_heading)
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf_m, 1.0)

        steering_gain = -2.38
        stanely_steering_gain = -2.38
        front_axle_vec = [np.cos(np.radians(car_heading)), -np.sin(np.radians(car_heading))]
        error_front_axle = np.dot([dx[current_index_yet], dy[current_index_yet]], front_axle_vec)
        k = 1.7
        theta_d = np.arctan2(k*error_front_axle, (now_velocity + 3.0))
        print("theta_d = ", theta_d)
        steering = (np.degrees(delta) * steering_gain) - (np.degrees(theta_d) * stanely_steering_gain)#* 0.6
        
        return steering

    def calc_wheels_utm(self, now_utm_x, now_utm_y, now_heading, front_WheelBase_dist=-1.30, back_WheelBase_dist=1.45):
        now_heading = np.deg2rad(now_heading)
        #front_WheelBase_utmX = (front_WheelBase_dist * np.cos(now_heading)) + now_utm_x
        #front_WheelBase_utmY = (front_WheelBase_dist * np.sin(now_heading)) + now_utm_y
        back_WheelBase_utmX = now_utm_x
        back_WheelBase_utmY = now_utm_y
        #return front_WheelBase_utmX, front_WheelBase_utmY, back_WheelBase_utmX, back_WheelBase_utmY
        return  back_WheelBase_utmX, back_WheelBase_utmY




        
if __name__ == '__main__':
	rospy.init_node('Remote',anonymous=True)
	controller = Point_tracking()
	rospy.spin()
		
		
