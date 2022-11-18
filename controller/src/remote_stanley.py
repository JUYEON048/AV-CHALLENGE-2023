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
        self.path_choose = str(1)  # input("1 : 대구 PG , 2 : 학교 주차장 주연이가 딴거, 3: 등판로, 4:일단 1자 등판로")
        #self.all_path_mode()

        self.L = 2.7 ###wheeeeeeeeel base ### [m] Wheel base of vehicle
        self.W = 1.72
        self.path_index = None
        self.r_L = 4.47  ### ioniq-4.47m, 1.82m
        self.r_W = 1.82

        self.map_yaw = 0
        self.start_index = 0

        self.rtk_sub = rospy.Subscriber('/Pose_messages', RTK, self.rtk_callback)
        self.car_sub = rospy.Subscriber('/car_messages', Car, self.car_callback)

        self.remote_pub = rospy.Publisher('remote_messages', Remote, queue_size = 10)

        self.final_cmd = Remote()
        self.final_cmd.steering = 0

        self.save_heading = None
        self.plot_flag = 0
        self.current_index = 0
        self.angle_sub = 0
        self.before_path_angle = 0
    
    def stanley_method(self):
        if self.angle_sub != 0:
            self.before_path_angle = self.angle_sub

        next_idx = 20

        x = self.PosX
        y = self.PosY

        if self.start_index == 0:
            dx_com_path = self.path_all_x[:1000]
            dy_com_path = self.path_all_y[:1000]
        else:
            dx_com_path = self.path_all_x[self.start_index: self.start_index + 30]
            dy_com_path = self.path_all_y[self.start_index: self.start_index + 30]

        dx = [x - ix for ix in dx_com_path[:]]
        dy = [y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        closest_error = min(d)  ## find min d index
        current_index_yet = d.index(closest_error)
        self.current_index = self.start_index + current_index_yet

        x_vect = self.path_all_x[self.current_index + next_idx] - self.path_all_x[self.current_index]
        y_vect = self.path_all_y[self.current_index + next_idx] - self.path_all_y[self.current_index]
        Angle_ = np.rad2deg(np.arctan(self.path_all_y[self.current_index + next_idx] / self.path_all_x[self.current_index + next_idx]))
        now_car_angle_ = np.rad2deg(np.arctan(self.path_all_y[self.current_index] / self.path_all_x[self.current_index]))
        Angle = np.round(Angle_, 4)
        now_car_angle = np.round(now_car_angle_, 4)
        angle_sub = (np.round(Angle - now_car_angle, 4)) * 1000

        if self.before_path_angle != 0:
            self.final_angle = self.angle_sub - self.before_path_angle
        else:
            self.before_path_angle = self.angle_sub
            self.final_angle = self.angle_sub - self.before_path_angle

        if current_index_yet > 10:
            self.start_index = self.current_index
        steering = self.final_angle * 100
        return steering
    def cal_dist(self, x1, y1, x2, y2, a, b):
        area = abs((x1 - a) * (y2 - b) - (y1 - b) * (x2 - a))
        AB = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        distance = area / AB
        return distance

    def stanley_1(self):
        x = self.PosX
        y = self.PosY

        if self.start_index == 0:
            dx_com_path = self.path_all_x[:600]
            dy_com_path = self.path_all_y[:600]
        else:
            dx_com_path = self.path_all_x[self.start_index: self.start_index + 50]
            dy_com_path = self.path_all_y[self.start_index: self.start_index + 50]

        print("  - ", len(self.path_all_x), len(self.path_all_y))
        dx = [x - ix for ix in dx_com_path[:]]
        dy = [y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        current_index_yet = d.index(min(d))
        self.now_position_index = current_index_yet + self.start_index
        self.current_index = self.now_position_index + 3  ## next_index
        map_yaw = self.yaw_all[self.current_index]
        steering = np.abs(map_yaw - self.heading)
        if map_yaw > self.heading:
            steering = -(steering)
        print("min distance = ", min(d))
        steering = steering + np.arctan(2.0 * dist / self.velocity)
        '''
        if (self.now_position_index >= 0 and self.now_position_index <= 310):
            steering = steering * 10
        if self.now_position_index == 10 or (
                self.now_position_index >= 30 and self.now_position_index <= 40) or self.now_position_index == 70:
            steering = steering * 0.75
        if (self.now_position_index >= 50 and self.now_position_index <= 60):
            steering = steering * 1.65
        if (self.now_position_index >= 80 and self.now_position_index <= 90):
            steering = steering + 1.5
        if (self.now_position_index >= 100 and self.now_position_index <= 170) or (
                self.now_position_index >= 190 and self.now_position_index <= 220) or self.now_position_index == 240 or self.now_position_index == 250:
            steering = steering - 10
        if self.now_position_index == 230:
            steering = steering - 1
        if (self.now_position_index >= 260 and self.now_position_index <= 290):
            steering = steering - 5.5
        if self.now_position_index == 300:
            steering = steering - 2
        '''

        dist = self.cal_dist(self.path_all_x[self.current_index], self.path_all_y[self.current_index],
                             self.path_all_x[self.now_position_index], self.path_all_y[self.now_position_index],
                             x, y)
        print(">> distance = ", dist)
        if dist > 0.8 :
           steering = steering * 1.4
        elif dist >0.5:
           steering = steering * 1.2

        steering = np.clip(steering, -29, 29)

        print("================================")
        print("car pos = ", self.now_position_index)
        print("path index = ", self.current_index)
        print("car target_yaw = ", map_yaw, ", pos_yaw = ", self.heading)
        print("car heading = ", self.heading)
        print("steering value = ", steering)
        print("Real Steering Value = ", self.real_steer[self.current_index])
        if current_index_yet > 10:  ## 꼭 필요함 아니면 차가 안갔는데 움직임
            self.start_index = self.current_index



        return steering

    def stanley_2(self):
        x = self.PosX
        y = self.PosY

        if self.start_index == 0:
            dx_com_path = self.path_all_x[:600]
            dy_com_path = self.path_all_y[:600]
        else:
            dx_com_path = self.path_all_x[self.start_index: self.start_index + 50]
            dy_com_path = self.path_all_y[self.start_index: self.start_index + 50]

        #dx = [x - ix for ix in dx_com_path[:]]
        #dy = [y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        current_index_yet = d.index(min(d))
        self.now_position_index = current_index_yet + self.start_index
        self.current_index = self.now_position_index + 15 ## next_index
        map_yaw = self.yaw_all[self.current_index]
        steering = np.abs(map_yaw - self.heading)
        if map_yaw > self.heading :
            steering = -(steering)
        print("min distance = ", d[5])
        dist = self.cal_dist(self.path_all_x[self.current_index], self.path_all_y[self.current_index],
                             self.path_all_x[self.now_position_index], self.path_all_y[self.now_position_index],
                             x, y)
        print(">> distance = ", dist)

        steering = steering + np.arctan(2.0 * dist / self.velocity)
        '''
        if self.now_position_index >= 255 and self.now_position_index < 263:
            steering = steering * 1.2
        elif self.now_position_index >= 263 and self.now_position_index < 297:
            steering = steering * 1.5
        elif self.now_position_index >= 298 and self.now_position_index < 334:
            steering = steering * 2.0
        elif self.now_position_index >= 334 and self.now_position_index < 345:
            steering = steering * 2.5
        elif self.now_position_index >= 345 and self.now_position_index < 359:
            steering = steering * 2.3
        elif self.now_position_index >= 359 and self.now_position_index < 364:
            steering = steering * 2.0
        elif self.now_position_index == 381:
            steering = steering  * 2.8
        elif self.now_position_index >= 364 and self.now_position_index < 382:
            steering = steering * 2.3
        elif self.now_position_index >= 382 and self.now_position_index < 394:
            steering = steering * 2.8
        elif self.now_position_index >= 394 and self.now_position_index < 432:
            steering = steering * 2.4
        elif self.now_position_index >= 432 and self.now_position_index < 500:
            steering = steering * 2.2222
            if np.abs(steering) < 24:
                steering = steering = (-23.5)
            if np.abs(steering) > 25:
                steering = steering = (-25)
        elif self.now_position_index >= 500 and self.now_position_index < 525:
            steering = steering * 2.22
        elif self.now_position_index >= 525 and self.now_position_index < 600:
            steering = steering * 2.2
        elif self.now_position_index >= 550 and self.now_position_index < 600:
            steering = steering * 2.0
        elif self.now_position_index >= 600 and self.now_position_index < 625:
            steering = steering * 1.8
        '''
        if dist > 0.8 :
            steering = steering * 1.4
        elif dist >0.5:
            steering = steering * 1.2

        steering = np.clip(steering, -32, 32)

        print("================================")
        print("car pos = ", self.now_position_index)
        print("path index = ", self.current_index)
        print("map_yaw = ", map_yaw, ", pos_yaw = ", self.heading)
        print("map - pos yaw = ", map_yaw - self.heading)
        print("실제 steering 값 = ", self.steering_real[self.now_position_index])
        print("steering value = ", steering)

        if current_index_yet > 10:   ## 꼭 필요함 아니면 차가 안갔는데 움직임
            self.start_index = self.x

        return steering

    def pure_pursuit(self):
        x = self.PosX
        y = self.PosY
        car_heading = self.heading

        fx, fy, self.rear_x, self.rear_y = self.calc_wheels_utm(x, y, car_heading)

        if self.start_index == 0:
            dx_com_path = self.path_all_x[:]
            dy_com_path = self.path_all_y[:]
        else:
            dx_com_path = self.path_all_x[self.start_index: self.start_index + 50]
            dy_com_path = self.path_all_y[self.start_index: self.start_index + 50]

        dx = [self.rear_x - ix for ix in dx_com_path[:]]
        dy = [self.rear_y - iy for iy in dy_com_path[:]]

        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        current_index_yet = d.index(min(d))

        Lf = (self.velocity * 0.237) + 4.29

        while Lf > d[current_index_yet]:
            if (current_index_yet + 1) >= len(self.path_all_x):
                print("Goal")
                break
            else:
                current_index_yet = current_index_yet + 1

        self.current_index = current_index_yet + self.start_index
        if current_index_yet > 10 :
            self.start_index = self.current_index
        if self.current_index <= self.start_index and self.start_index != 0:
            self.current_index = self.current_index + 4

        car_front_point_x = fx - self.rear_x
        car_front_point_y = fy - self.rear_y
        map_point_x = self.path_all_x[self.current_index] - self.rear_x
        map_point_y = self.path_all_y[self.current_index] - self.rear_y

        target_yaw = np.rad2deg(math.atan2(map_point_y, map_point_x))
        pos_yaw = np.rad2deg(math.atan2(car_front_point_y, car_front_point_x))

        map_yaw = target_yaw - pos_yaw

        steering = map_yaw

        if np.abs(map_yaw) > 20:
            steering = steering * 1.5
        elif np.abs(map_yaw) > 10:
            steering = steering * 1.2

        steering = np.clip(steering, -439, 439)

        print("================================")
        print("car pos = ", self.start_index)
        print("path index = ", self.current_index)
        print("car heading = ", car_heading)
        print("self.heading = ", self.heading)
        print("car target_yaw = ", target_yaw, ", pos_yaw = ", pos_yaw)
        print("target_yaw - car_pos_yaw = ", map_yaw)
        print("steering value = ", steering)

        return steering

    def calc_wheels_utm(self, now_utm_x, now_utm_y, now_heading, front_WheelBase_dist=-1.30, back_WheelBase_dist=1.45):

        now_heading = np.deg2rad(now_heading)
        front_WheelBase_utmX = (front_WheelBase_dist * np.cos(now_heading)) + now_utm_x
        front_WheelBase_utmY = (front_WheelBase_dist * np.sin(now_heading)) + now_utm_y

        back_WheelBase_utmX = (back_WheelBase_dist * np.cos(now_heading)) + now_utm_x
        back_WheelBase_utmY = (back_WheelBase_dist * np.sin(now_heading)) + now_utm_y

        return front_WheelBase_utmX, front_WheelBase_utmY, back_WheelBase_utmX, back_WheelBase_utmY

    def car_callback(self, car):
        self.velocity = car.velocity
        self.steering_real = car.steering

    def rtk_callback(self, RTK):
        self.PosX = RTK.utm_x
        self.PosY = RTK.utm_y
        self.heading = RTK.heading
        
        #with open('./wmgpdwmgpd22222.csv', 'a') as f:
        #    f.write(str(self.PosX)+ ',' + str(self.PosY) + ',' + str(self.heading)  + str("\n"))
        #self.steering = self.stanley_1()
        '''
        if self.heading == '':
            if self.save_heading == None:
                self.heading = self.yaw_all[0]
            else:
                self.heading = float(self.save_heading) 
        else:
            self.save_heading = float(self.heading)
            self.heading = float(self.heading)
        '''

        with open('RTK_sujung_hu.csv', 'a') as f:
            f.write(str(self.PosX)+ ',' + str(self.PosY) + ',' + str(self.heading) + ',' + str(self.steering_real) + str("\n"))
            #f.write(str(self.PosX)+ ',' + str(self.PosY) + ',' + str(self.heading) +str("\n"))

        '''
        if self.path_choose == str(1):
            if (self.current_index > 360 and self.current_index < 670) or (self.current_index > 1150):
                if (self.current_index > 560 and self.current_index < 660) or (self.current_index > 1240 and self.current_index < 1460):
                    self.steering = -23
                else:
                    self.steering = self.stanley_()
            else:
                self.steering = self.pure_pursuit()
        else:
            self.steering = self.stanley_()
            self.steering = self.pure_pursuit_220628()  
        '''
        #self.final_cmd.steering = self.steering
        #self.remote_pub.publish(self.final_cmd)
        
			
if __name__ == '__main__':
	rospy.init_node('Remote',anonymous=True)
	controller = Algorithm_calc()
	rospy.spin()
		
		
