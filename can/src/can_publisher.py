#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from mi_msgs.msg import *
from can_msgs.msg import Frame
import numpy as np

class Can_publisher(object):
	def __init__(self):
		self.remote_sub = rospy.Subscriber('control_messages',Control,self.control_callback)
		#self.SCH_LGIT_sub = rospy.Subscriber('SCH_LGIT_messages',SCH_LGIT,self.SCH_LGIT_callback)
		self.car_sub = rospy.Subscriber('car_messages',Car,self.SCH_car_callback)
		self.can_pub = rospy.Publisher('sent_messages', Frame, queue_size = 10)
		self.heartbeat = 0
		self.steering_possible_range = 120
		self.id_list = ['150','152','303']
		self.send_data = {}
		self.can_init()
		self.final_cmd = Control()
		#self.SCH_LGIT_state = SCH_LGIT()
	
	def SCH_car_callback(self,data):

		heading_lg = (-90*(1-data.left_right)) + data.heading*data.left_right
		heading = ((heading_lg * -1) - 90)
		heading = round(heading,2)*10
		X = (-1*(round(data.PosX,2) + 1.35*np.cos(np.radians(data.heading)))*data.left_right) * 1000
		Y = (round(data.PosY,2) - 1.35*np.sin(np.radians(data.heading))) * 1000
		print("heading,X,Y",data.heading,data.PosX,data.PosY)
		heading_1 = np.int16(heading)
		heading_2 = np.uint8((heading_1&0xff00)>>8)
		heading_3 = np.uint8(heading_1&0x00ff)
		self.send_data['303'].data[2] = heading_3	
		self.send_data['303'].data[3] = heading_2
		
		X_1 = np.int16(X)
		X_2 = np.uint8((X_1&0xff00)>>8)
		X_3 = np.uint8(X_1&0x00ff)
		self.send_data['303'].data[4] = X_3	
		self.send_data['303'].data[5] = X_2
		
		Y_1 = np.int16(Y)
		Y_2 = np.uint8((Y_1&0xff00)>>8)
		Y_3 = np.uint8(Y_1&0x00ff)
		self.send_data['303'].data[6] = Y_3	
		self.send_data['303'].data[7] = Y_2
		

	def SCH_LGIT_callback(self,data):
		self.SCH_LGIT_state = data
		self.send_data['303'].data[0] = np.uint8(data.status)
		if data.status == 4:
			self.send_data['303'].data[1] = 5
		else:
			self.send_data['303'].data[1] = 5
			
	def control_callback(self,data):
		self.final_cmd = data
		self.can_publish()
			
	def can_init(self):
		for id_ in self.id_list:
			self.send_data[id_] = Frame()
			self.send_data[id_].is_rtr = False
			self.send_data[id_].is_extended = False
			self.send_data[id_].is_error = False
			self.send_data[id_].dlc = 8
			self.send_data[id_].id = int(id_,16)	
			self.send_data[id_].data = [0,0,0,0,0,0,0,0]
	
	def heartpumping(self):
		self.heartbeat += 1
		self.send_data['150'].data[1] = np.uint8(self.heartbeat)
		self.send_data['150'].data[5] = np.uint8(self.steering_possible_range)

	def accel(self):
		#print("self.final_cmd.accel",self.final_cmd.accel)
		accel_1 = np.int16(self.final_cmd.accel)
		accel_2 = np.uint8((accel_1&0xff00)>>8)
		accel_3 = np.uint8(accel_1&0x00ff)
		self.send_data['152'].data[0] = accel_3
		self.send_data['152'].data[1] = accel_2
		
		'''
		self.send_data['152'].data[0] = np.uint8(self.final_cmd.accel%256)
		self.send_data['152'].data[1] = np.uint8(self.final_cmd.accel/256)
		'''
	def brake(self):
		#print("self.final_cmd.brake",self.final_cmd.brake)
		brake_1 = np.int16(self.final_cmd.brake)
		brake_2 = np.uint8((brake_1&0xff00)>>8)
		brake_3 = np.uint8(brake_1&0x00ff)
		self.send_data['152'].data[2] = brake_3
		self.send_data['152'].data[3] = brake_2
		#self.send_data['152'].data[2] = np.uint8(self.final_cmd.brake%256)
		#self.send_data['152'].data[3] = np.uint8(self.final_cmd.brake/256)
		
	def steering(self):
		#rospy.loginfo("steering = %d"%self.final_cmd.steering)
		steering_1 = np.int16(self.final_cmd.steering)
		steering_2 = np.uint8((steering_1&0xff00)>>8)
		steering_3 = np.uint8(steering_1&0x00ff)
		self.send_data['152'].data[4] = steering_3	
		self.send_data['152'].data[5] = steering_2
		'''
		if(self.final_cmd.steering >= 0):
			self.send_data['152'].data[4] = np.uint8(self.final_cmd.steering%256)
			self.send_data['152'].data[5] = np.uint8(self.final_cmd.steering/256)		
		elif(self.final_cmd.steering < 0):
			self.send_data['152'].data[4] = np.uint8(255 - abs(self.final_cmd.steering)%256)
			self.send_data['152'].data[5] = np.uint8(255 - abs(self.final_cmd.steering)/256)	
		'''
	def gear_shift(self):
		#print("self.final_cmd.gear",self.final_cmd.gear)
		self.send_data['152'].data[6] = np.uint8(self.final_cmd.gear)
		
	def timer_callback(self,event):
		#if self.SCH_LGIT_state.status >= 1:
		#self.can_pub.publish(self.send_data['303'])
	#	self.can_pub.publish(self.send_data['152'])
		self.can_publish() ##timer_callback 함수에서 주기적으로 실행되어야 하는 함수 can_pulish()

	def can_publish(self): 
		self.heartpumping() ###heartbeat Command
		self.accel() ###accel Command
		self.brake() ###brake Command
		self.steering() ###steering Command
		self.gear_shift() ###gear_shift Command
		self.can_pub.publish(self.send_data['150'])
		self.can_pub.publish(self.send_data['152'])
		#print(self.send_data['303'])
		#self.can_pub.publish(self.send_data['303'])
	
		#print("can.id(702) = ",self.send_data['702'].data)
		#self.can_pub.publish(self.send_data['702']) ###can id 702 publish

if __name__ == '__main__':
	
	rospy.init_node('can_publisher',anonymous=True)
	can_publisher = Can_publisher()
	timer = rospy.Timer(rospy.Duration(0.03), can_publisher.timer_callback)
	rospy.spin()
	timer.shutdown()
	
