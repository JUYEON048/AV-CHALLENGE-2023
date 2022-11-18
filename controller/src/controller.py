#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import array
import copy
from accel_steer_controller import *
from mi_msgs.msg import *

class Controller(object):
	
	def __init__(self):
		self.target_value = Remote()
		self.final_cmd = Control()
		self.car_state = Car()
		self.car_state.velocity = 0
		self.remote_control_cmd = Remote()
		self.sub_val = 999
		self.change_gear_flag = False
		self.target_value.velocity = 0	
		self.target_value.steering = 0
		self.target_value.gear = 0
		self.current_velocity_value = 0
		self.prev_value = Control()
		self.prev_value.accel = 0
		self.prev_value.steering = 0
		self.prev_value.brake = 0
		self.A_S_controller = Accel_Steer_Controller()
		self.A_S_controller.__init__()
		self.accel_max_change = rospy.get_param("~/controller/accel_max_change")
		self.steering_max_change = rospy.get_param("~/controller/steering_max_change")
		self.brake_max_change = rospy.get_param("~/controller/brake_max_change")
		self.accel_limiter_max = rospy.get_param("~/controller/accel_limiter_max")
		self.accel_limiter_min = rospy.get_param("~/controller/accel_limiter_min")
		self.brake_limiter_max = rospy.get_param("~/controller/brake_limiter_max")
		self.brake_limiter_min = rospy.get_param("~/controller/brake_limiter_min")
		self.brake_gain = rospy.get_param("~/controller/brake_gain")
		self.steering_limiter_max = rospy.get_param("~/controller/steering_limiter_max")
		self.steering_limiter_min = rospy.get_param("~/controller/steering_limiter_min")
		self.control_pub = rospy.Publisher('control_messages', Control, queue_size = 10)
		self.remote_sub = rospy.Subscriber('remote_messages',Remote,self.remote_callback)
		self.car_sub = rospy.Subscriber('car_messages',Car,self.car_callback)

		
	def remote_callback(self, data):
		self.remote_control_cmd = data

	def car_callback(self, data):
		self.car_state = data
		self.current_velocity_value = self.car_state.velocity

		if self.remote_control_cmd.gear == 202:
			print("Not ready for auto mode")
			exit()
		else:
			self.target_value = self.remote_control_cmd
			
		if self.car_state.gear == 6:
			self.change_gear_flag = True
		elif self.car_state.gear == 5 and self.change_gear_flag:
			self.sub_val = 999
		
		sub_val = self.target_value.velocity - self.current_velocity_value
		if self.target_value.velocity == 0.0:
			self.calc()
		else:
			if sub_val < 0:
				self.calc()
				self.sub_val = sub_val
			else:
				if self.sub_val <= 0 and sub_val == self.target_value.velocity :
					self.calc()
					#pass
				else:
					if self.sub_val < self.target_value.velocity and sub_val == self.target_value.velocity:
						self.calc()
						#pass
					else:
						self.calc()
						self.sub_val = sub_val
		self.control_publish()
		#self.sub_val = sub_val

	
	def brake(self):
		
		if(self.target_value.velocity == 0.0):
			#self.brake_max_change = rospy.get_param("~brake_max_change") + 200 
			self.brake_cmd = 15000
			self.accel_cmd = 0
		elif(self.target_value.velocity > 0.0):
			#self.brake_max_change = rospy.get_param("~brake_max_change")
			self.brake_cmd = 0
			self.accel_cmd = self.accel_cmd 
		print(self.target_value.velocity, self.accel_cmd )
	
	def brake_accel_to_gear_shift(self):
		if(self.accel_cmd < 0 ):
			self.brake_cmd = abs(self.accel_cmd) * self.brake_gain
			self.accel_cmd = 0
			
	def velocity(self):
		self.accel_cmd = self.A_S_controller.accel_control(self.target_value.velocity,self.current_velocity_value)
		self.accel_cmd = np.clip(self.accel_cmd,self.accel_limiter_min,self.accel_limiter_max) ###Accel limiter
		self.brake()
		self.brake_accel_to_gear_shift()
		self.accel_cmd = self.A_S_controller.rate_limiter(self.accel_cmd,self.prev_value.accel,self.accel_max_change)
		self.final_cmd.accel = self.accel_cmd
		self.prev_value.accel= copy.deepcopy(self.final_cmd.accel)
		#################################brake######################################
		self.brake_cmd = self.A_S_controller.rate_limiter(self.brake_cmd,self.prev_value.brake,self.brake_max_change)
		self.brake_cmd = np.clip(self.brake_cmd,self.brake_limiter_min, self.brake_limiter_max) ###Brake limiter
		self.final_cmd.brake = self.brake_cmd
		self.prev_value.brake = copy.deepcopy(self.final_cmd.brake)


	def steering(self):
		self.steering_cmd = self.A_S_controller.rate_limiter(self.target_value.steering,self.prev_value.steering,self.steering_max_change)
		self.steering_cmd = np.clip(self.steering_cmd, self.steering_limiter_min, self.steering_limiter_max) ###Steering limiter
		self.final_cmd.steering = self.steering_cmd
		self.prev_value.steering = copy.deepcopy(self.final_cmd.steering)

	
	def gear_shift(self):
		self.final_cmd.gear = np.uint8(self.target_value.gear)
	
	def control_publish(self):
		self.control_pub.publish(self.final_cmd)

	def calc(self):
		self.velocity() ###velocity Command
		self.steering() ###steering Command
		self.gear_shift() ###gear_shift Command
		
if __name__ == '__main__':
	rospy.init_node('controller',anonymous=True)
	controller = Controller()
	rospy.spin()
	
