#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pynmea2
from socket import *
from pyproj import Proj, transform
from mi_msgs.msg import *



class RTK_Class(object):
    def __init__(self):
        self.rtk_msg = RTK()
        self.proj_WGS84 = Proj("+proj=latlong +datum=WGS84 +ellps=WGS84") #Proj(init='epsg:4326')
        self.proj_UTM =  Proj("+proj=utm +zone=52 +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
        self.RTK_msg_pub = rospy.Publisher('Pose_messages', RTK, queue_size = 10)

    def parseGPS(self, event=None):
        ssock = socket(AF_INET, SOCK_STREAM)
        ssock.bind(("10.10.0.31", 9008))
        ssock.listen(0)

        print("!! Waiting request messages from client!!")
        conn, addr = ssock.accept()

        print("!! Accept request messages from client!!")
        while True:
            RTK_NMEA = conn.recv(65535)#(65535)
            rtk_mnea = RTK_NMEA.decode("utf-8", "ignore")
            
            if len(rtk_mnea) > 120:
                continue

            elif 'RMC' in rtk_mnea :
                msg = pynmea2.parse(RTK_NMEA)
                
                lat = (float(msg.lat[2:])/60) + float(msg.lat[:2]) 
                lon = (float(msg.lon[3:])/60) + float(msg.lon[0:3]) 
                self.rtk_msg.lon = lon
                self.rtk_msg.lat = lat
                # wgs84 to UTM
                self.rtk_msg.utm_x, self.rtk_msg.utm_y = transform(self.proj_WGS84, self.proj_UTM, lon, lat) #Transform WGS84 to UTM
                self.rtk_msg.header.stamp = rospy.Time.now()
                self.rtk_msg.header.frame_id = "rtk"
                self.rtk_msg.heading = msg.data[7]
                self.RTK_msg_pub.publish(self.rtk_msg)
                print(self.rtk_msg, '\n', '----------')
            else:
                pass
        ssock.close()


if __name__=='__main__':
    rospy.init_node('RTK_parser')
    rc = RTK_Class()
    rc.parseGPS()
    #rospy.Timer(rospy.Duration(1.0/50.0), rc.parseGPS)
    rospy.spin()
    
    