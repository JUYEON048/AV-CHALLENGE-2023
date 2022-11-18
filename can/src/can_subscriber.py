#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import array
from can_parsing import *
from can_msgs.msg import Frame
from mi_msgs.msg import *

class Can_subscriber(object):
	
	def __init__(self):
		self.id_list = ['51','52','53','1B0','1C0','300','301','302']
		self.send_data = {}
		self.can_init()
		self.can_parser = Can_parsing()
				
		self.car_pub = rospy.Publisher('car_messages', Car, queue_size = 10)
		self.trigger_sub = rospy.Subscriber('trigger_pub_messages',Trigger,self.trigger_callback)
		self.can_sub = rospy.Subscriber('received_messages',Frame,self.can_callback)
		self.imu_sub = rospy.Subscriber('IMU_MI_messages',imu_mi,self.imu_callback)
		
	def can_init(self):
		for id_ in self.id_list:
			self.send_data[id_] = Frame()
			self.send_data[id_].is_rtr = False
			self.send_data[id_].is_extended = False
			self.send_data[id_].is_error = False
			self.send_data[id_].dlc = 8
			self.send_data[id_].id = int(id_,16)	
			self.send_data[id_].data = [0,0,0,0,0,0,0,0]
	
	def trigger_callback(self,data):
		self.can_parser.initializer(data.initialization)
		self.can_parser.location_update(data.location)
		
	def imu_callback(self,data):
		self.can_parser.imu_parser(data)
	

	def can_callback(self, data):
		if(self.send_data['51'].id == data.id):
			self.send_data['51'].data = map(ord,data.data)
			self.send_data['51'].header = data.header
			self.can_parser.car_SCH_parser_51(self.send_data)
		if(self.send_data['52'].id == data.id):
			self.send_data['52'].data = map(ord,data.data)
			self.send_data['52'].header = data.header
			self.can_parser.car_SCH_parser_52(self.send_data)
		if(self.send_data['53'].id == data.id):
			self.send_data['53'].data = map(ord,data.data)
			self.send_data['53'].header = data.header
			self.can_parser.car_SCH_parser_53(self.send_data)
		if(self.send_data['300'].id == data.id):
			self.send_data['300'].data = map(ord,data.data)
			self.send_data['300'].header = data.header
			self.can_parser.car_LGIT_parser_300(self.send_data)
		if(self.send_data['301'].id == data.id):
			self.send_data['301'].data = map(ord,data.data)
			self.send_data['301'].header = data.header
			self.can_parser.car_LGIT_parser_301(self.send_data)
		if(self.send_data['302'].id == data.id):
			self.send_data['302'].data = map(ord,data.data)
			self.send_data['302'].header = data.header
			self.can_parser.car_LGIT_parser_302(self.send_data)
		if(self.send_data['1B0'].id == data.id):
			self.send_data['1B0'].data = map(ord,data.data)
			self.send_data['1B0'].header = data.header
			self.can_parser.sonar_parser_1B0(self.send_data)
		if(self.send_data['1C0'].id == data.id):
			self.send_data['1C0'].data = map(ord,data.data)
			self.send_data['1C0'].header = data.header
			self.can_parser.sonar_parser_1C0(self.send_data)
	
	def timer_callback(self,event):
		self.car_pub.publish(self.can_parser.SCH_car_pub())
		
if __name__ == '__main__':
	
	rospy.init_node('can_subscriber',anonymous=True)
	can_subscriber = Can_subscriber()
	timer = rospy.Timer(rospy.Duration(0.01), can_subscriber.timer_callback)
	rospy.spin()
	timer.shutdown()
