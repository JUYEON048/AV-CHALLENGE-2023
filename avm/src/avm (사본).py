#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cmath import isnan
#from tkinter import N
#from tkinter.messagebox import NO
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mi_msgs.msg import *
import time
from std_msgs.msg import Int32


class AVM(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.avm_img_raw_sub = rospy.Subscriber("/camera/image_color", Image, self.avm_img_raw_callback)
        self.line_change_sub = rospy.Subscriber("line_change", Int32, self.line_change_callback)
        self.line_state_pub = rospy.Publisher("line_state", Line, queue_size = 10)
        #self.rkt_sub = rospy.Subscriber("/Pose_messages", RTK, self.rtk_callback)
        #self.rkt_sub = rospy.Subscriber("/RTK_messages", RTK, self.rtk_callback)
        #self.obj_state_sub = rospy.Subscriber("/Obj_state", Obj_state, self.obj_state_callback)
        self.dist_from_center = Line()
        #Config
        #self,corner_count = 0
        self.corner_flag = False
        self.line_chage_state = 0
        self.left_right_sub = 0
        self.start_flag = False
        self.start_time = time.time()
        self.now_utmX = 0
        self.now_utmY = 0          
        self.obj_rightLane_flag = False
        self.after_corner = False

        self.bright_rate = 180 #100!  #160 #150
        self.threshod_rate = 50 #70

            # --- parameter set ------
            # set1) cloud >> bright_rate : 150, threshod_rate : 80

        self.rho = 1
        self.theta = np.pi/180
        self.threshold = 50 #50
        self.min_line_len = 10 #15 #12 #10
        self.max_line_gap = 15 #18 #15
        self.x1_list_l = []
        self.x2_list_l = []
        self.y1_list_l = []
        self.y2_list_l = []
        self.x1_list_r = []
        self.x2_list_r = []
        self.y1_list_r = []
        self.y2_list_r = []

    def line_change_callback(self, data):
        self.line_chage_state = data.data

    def avm_img_raw_callback(self,data):
        '''if self.start_flag:
            now_time = time.time()
            during_time = now_time - self.start_time
            if during_time > 5:
                self.start_flag = False'''

        # for line change ---------------->
        
        if self.line_chage_state == 0:
            pass
        elif self.line_chage_state == 1: #Right to Left
            #$ rostopic pub /line_change std_msgs/Int32 1
            self.dist_from_center.header.stamp = rospy.Time.now()
            self.dist_from_center.header.frame_id = "lane"
            self.dist_from_center.data = 11
            self.line_state_pub.publish(self.dist_from_center)
            time.sleep(2.0) #3sec
            #print("R2L", self.line_chage_state)
            self.line_chage_state = 0
            return
        elif self.line_chage_state == 2:
            #$ rostopic pub /line_change std_msgs/Int32 2
            self.dist_from_center.header.stamp = rospy.Time.now()
            self.dist_from_center.header.frame_id = "lane"
            self.dist_from_center.data = -11
            self.line_state_pub.publish(self.dist_from_center)
            time.sleep(2.0) #3sec
            #print("L2R", self.line_chage_state)
            self.line_chage_state = 0
            return
        # <--------------------
        
        avm_img_raw = self.bridge.imgmsg_to_cv2(data, 'bgr8') # convert rostopic to img
    #    self.set_roi(avm_img_raw)        
    #def set_roi(self,img):
        # resize image
        img = cv2.resize(avm_img_raw, dsize=(1280,512), interpolation=cv2.INTER_AREA)
        
        # transform to birds view
        #pts1 = np.float32([[500,306],[820,306],[290,406],[1020,406]]) #좌상->좌하->우상->우하
        pts1 = np.float32([[500,306],[820,306],[290,396],[1020,396]]) #좌상->좌하->우상->우하
        pts2 = np.float32([[10,10],[100,10],[10,300],[100,300]])
        M = cv2.getPerspectiveTransform(pts1, pts2)
        birdview_img = cv2.warpPerspective(img, M, (110,310))
        result = np.copy(birdview_img)
        birdview_img = cv2.add(birdview_img, self.bright_rate) #print(birdview_img.shape) #(310, 110, 3)
        birdview_img = np.clip(birdview_img, 0, 255)
        
        # filtering white colors
        thresholds = birdview_img[:, :, 0] < self.threshod_rate #100
        birdview_img[thresholds] = [0, 0, 0]
        thresholds = birdview_img[:, :, 1] < self.threshod_rate
        birdview_img[thresholds] = [0, 0, 0]
        thresholds = birdview_img[:, :, 2] < self.threshod_rate
        birdview_img[thresholds] = [0, 0, 0]
        edge_img_bv = cv2.Canny(birdview_img, 100, 255)

        roi_left = np.copy(edge_img_bv[:, :35])
        roi_right = np.copy(edge_img_bv[:, 75:])

        # extract white pixel 
        if self.obj_rightLane_flag:
            pass
        else:
            lines_l = cv2.HoughLinesP(roi_left, self.rho, self.theta, self.threshold, np.array([]), minLineLength=self.min_line_len, maxLineGap=self.max_line_gap)
            if lines_l is not None:
                for i in range(len(lines_l)):
                    for x1_,y1_,x2_,y2_ in lines_l[i]:
                        cv2.line(result,(x1_,y1_),(x2_,y2_),(255,0,0),2)
                        self.x1_list_l.append(x1_)
                        self.x2_list_l.append(x2_)
                        self.y1_list_l.append(y1_)
                        self.y2_list_l.append(y2_)      
            else:
                #print("DO NOT DETECTION LINES")
                pass
            x_left = (np.mean(self.x1_list_l)+np.mean(self.x2_list_l))/2
            y_left = (np.mean(self.y1_list_l)+np.mean(self.y2_list_l))/2

        
        if self.start_flag:
            pass
        
        else:
            lines_r = cv2.HoughLinesP(roi_right, self.rho, self.theta, self.threshold, np.array([]), minLineLength=self.min_line_len, maxLineGap=self.max_line_gap)
            if lines_r is not None:
                for i in range(len(lines_r)):
                    for x1_,y1_,x2_,y2_ in lines_r[i]:
                        cv2.line(result[:, 75:],(x1_,y1_),(x2_,y2_),(255,0,0),2)
                        self.x1_list_r.append(x1_)
                        self.x2_list_r.append(x2_)
                        self.y1_list_r.append(y1_)
                        self.y2_list_r.append(y2_)   
            else:
                pass #print("DO NOT DETECTION LINES")
            x_right = (np.mean(self.x1_list_r)+np.mean(self.x2_list_r))/2 +75
            y_right = (np.mean(self.y1_list_r)+np.mean(self.y2_list_r))/2 +75



        if isnan(x_left) or isnan(y_left):
            if isnan(x_right) or isnan(y_right):
                pass #print("---- Not Detected Lane ----")
            else:
                if(len(self.x1_list_r) > 13):
                    self.dist_from_center.data = int(55 - x_right) + 5
                else:
                    self.dist_from_center.data = int(55 - x_right)
                #self.dist_from_center.data = int(55 - x_right)
                del self.x1_list_r[:]
                del self.x2_list_r[:]
                del self.y1_list_r[:]
                del self.y2_list_r[:]
                print("Dist point [R] from center >> ", int(55 - x_right))
                cv2.line(result,(int(x_right),int(y_right)),(int(x_right),int(y_right)),(0,0,255),5)
        
        elif isnan(x_right) or isnan(y_right):
            if isnan(x_left) or isnan(y_left):
                pass #print("---- Not Detected Lane ----")
            else:
                if(len(self.x1_list_l) > 13):
                    self.dist_from_center.data = int(55 - x_left) - 3
                else:
                    self.dist_from_center.data = int(55 - x_left)
                #self.dist_from_center.data = int(55 - x_left)
                del self.x1_list_l[:]
                del self.x2_list_l[:]
                del self.y1_list_l[:]
                del self.y2_list_l[:]
                print("Dist point [L] from center >> ", int(55 - x_left))
                cv2.line(result,(int(x_left),int(y_left)),(int(x_left),int(y_left)),(0,0,255),5)

        else:
            #print(int(55 - x_left), int(x_right - 55), int(55 - x_left) - int(x_right - 55))
            self.left_right_sub = int(55 - x_left) - int(x_right - 55)
            x_mid = int((x_left + x_right)/2)
            if(len(self.x1_list_r) > 13):
                    self.dist_from_center.data = int(55 - x_mid) + 5
            else:
                self.dist_from_center.data = int(55 - x_mid)
            #self.dist_from_center.data = int(55 - x_mid)
            del self.x1_list_r[:]
            del self.x2_list_r[:]
            del self.y1_list_r[:]
            del self.y2_list_r[:]
            del self.x1_list_l[:]
            del self.x2_list_l[:]
            del self.y1_list_l[:]
            del self.y2_list_l[:]
            y_mid = int((y_left + y_right)/2)
            print("Dist point from center >> ",55 - x_mid)
            cv2.line(result,(x_mid,y_mid),(x_mid,y_mid),(0,0,255),5)

        #self.corner_flag = True
            #if float(445916.0) <= self.now_utmX  and self.now_utmX <= float(445920.0):
            #    if float(3944850.0) <= self.now_utmY  and self.now_utmY <= float(3944920.0): 
        if self.corner_flag:
            if 445918 <= self.now_utmX  and self.now_utmX <= 445922:
                if 394850 <= self.now_utmY  and self.now_utmY <= 394920:
                    print(self.now_utmX, self.now_utmY)
                    self.after_corner = True
                    self.corner_flag = False
                    print("[ON] Corner Flag Off -- using RTK")
                else:
                    if self.after_corner:
                        self.after_corner = False
                        #self.dist_from_center.data = -10   
                        print("[OFF] Corner Flag Off -- using RTK", -10)
            else:
                if self.after_corner:
                    self.after_corner = False
                    #self.dist_from_center.data = -10 
                    print("[OFF] Corner Flag Off -- using RTK", -10)
            


        if self.dist_from_center.data <= -14:
            self.corner_flag = True
            print("[ON ] Corner Flag On \n")

        elif -1 <= self.dist_from_center.data and self.dist_from_center.data <= 1 and self.corner_flag:
            self.corner_flag = False
            #self.corner_count = self.corner_count + 1 
            print("[OFF] Corner Flag Off \n")
        else:
            pass


        if np.abs(self.left_right_sub) >= 20 and self.corner_flag:
            self.dist_from_center.data = self.left_right_sub
            print("!!! Left-Right :: steering ", self.left_right_sub)
        else:
            pass
    

        cv2.line(result,(55,0),(55,310),(0,255,0),1) #image centor pixel
        # image show
        cv2.imshow("[2]]",result)
        #cv2.imshow("[1]]",img)
        skey = cv2.waitKey(1)
        
        print('-'*10)
        self.dist_from_center.header.stamp = rospy.Time.now()
        self.dist_from_center.header.frame_id = "lane"
        self.line_state_pub.publish(self.dist_from_center)

    

if __name__ == '__main__':
    rospy.init_node('avm',anonymous=True)
    avm = AVM()
    rospy.spin()
