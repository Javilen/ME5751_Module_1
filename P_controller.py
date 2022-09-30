#!/usr/bin/python
# -*- coding: utf-8 -*-

from calendar import c
from E160_state import *
from E160_robot import *
import math
import time


class P_controller:

	def __init__(self, robot, logging = False):
		self.robot = robot  # do not delete this line
		self.kp = 3#0  # k_rho 0.5
		self.ka = 5#0  # k_alpha  5
		self.kb = 0.5#0  # k_beta  0.1
		self.finish = 1
		#self.kp = 5  #5
		#self.kb = -1  # -1
		#self.ka = 2   # 2
		self.logging = logging

		if(logging == True):
			#self.robot.make_headers(['pos_X','posY','posZ','vix','viy','wi','vr','wr'])
			#self.robot.make_headers(['pos_X','rho', 'd_theta', 'alpha', 'beta'])
			self.robot.make_headers([ 'do it now' , 'rho' ])

		self.set_goal_points()

	#Edit goal point list below, if you click a point using mouse, the points programmed
	#will be washed out
	def set_goal_points(self):
		# here is the example of destination code
		
		self.robot.state_des.add_destination(x=100,y=0,theta=0.25)    #goal point 1
		self.robot.state_des.add_destination(x=90,y=30,theta=1.57) #goal point 2
		self.robot.state_des.add_destination(x=-50,y=30,theta=-1.57) #goal point 3
		self.robot.state_des.add_destination(x=190,y=30,theta=1.57) #goal point 4
		self.robot.state_des.add_destination(x=0,y=0,theta=0) #goal point 5


	def track_point(self):

		# All d_ means destination

		(d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration

		# All c_ means current_

		(c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
		(c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
		(c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration


		# Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
		# Deas edit time!~!!
		rho=math.sqrt((d_posX-c_posX)**2+(d_posY-c_posY)**2)
		omega=math.atan2(d_posY-c_posY,d_posX-c_posX)
		alpha=omega-c_theta
		#beta=omega-d_theta # omega-d_theta
		if (- math.pi/2 < alpha <= math.pi/2):
			beta=omega-d_theta # omega-d_theta
			c_v= self.kp*rho
			#self.robot.log_data(['fart_ass'])
		else:
			beta= -(omega - 1.57) + d_theta  #omega - 1.57
			alpha =  c_theta + beta # - c_theta - beta
			#beta=-(omega-d_theta)
			#c_v= -self.kp*rho

		if (abs(beta) > 1.57):
			self.robot.log_data([ beta ])
			

		# set new c_v = k_rho*rho, c_w = k_alpha*alpha + k_beta*beta

		#c_v= self.kp*rho

		if c_v > 40: c_v=40


		c_w=self.ka*alpha + self.kb*beta
		
		if abs(c_w) > 15: # saturation filter
			if abs(c_w) == c_w:
				c_w = 15
			else:
				c_w = -15
		
		#c_v = 25 #randomly assigned c_v and c_w for demonstration purpose
		#c_w = 0 #1.57

		# self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
		self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
		
		# you need to write code to find the wheel speed for your c_v, and c_w, the program won't calculate it for you.
		# my fun attempt Deas
		phi_l = (1/3)*c_v +4*c_w
		phi_r = (1/3)*c_v -4*c_w
		self.robot.send_wheel_speed(float("{:.1f}".format(phi_l)),float("{:.1f}".format(phi_r))) #unit rad/s phi_l = 6.0,phi_r = 6.0


		# use the following to log the variables, use [] to bracket all variables you want to store
		# stored values are in log folder
		if self.logging == True:
			#self.robot.log_data([c_posX,c_posY,c_theta,c_vix,c_viy,c_wi,c_v,c_w])
			#self.robot.log_data([ c_posX , rho , d_theta, alpha, beta ])
			self.robot.log_data([ omega ])

		if abs(rho) < self.finish: #you need to modify the reach way point criteria  if abs(c_posX - d_posX) < 80:
			if(self.robot.state_des.reach_destination()): 
				print("final goal reached")
				self.robot.set_motor_control(.0, .0)  # stop the motor
				self.robot.send_wheel_speed(.0, .0)
				return True
			else:
				print("one goal point reached, continute to next goal point")
		
		return False