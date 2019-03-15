#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int16
import nav_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf
import math
import random
import time
import datetime
import path_finder as pf
import numpy as np
import utils
from std_msgs.msg import Int16MultiArray

class bot (object):
	def __init__(self, id_):
		self.id = id_
		self.name = "/robot_"+str(id_)
		self.priority = 0

		self.present_dir = 180
		self.target_dir=0

		self.odom_t = "/barcode_scanner_data"
		self.cmd_vel_t = "/cmd_vel"
		self.diagnostics_t = self.name + "/diagnostics"

		self.odom = rospy.Subscriber(self.odom_t,Int16MultiArray,self.updatePresentState,queue_size=10) #queue size?
		self.cmd_vel = rospy.Publisher(self.cmd_vel_t, geometry_msgs.msg.Twist,queue_size=10)
		self.diagnostic = rospy.Publisher(self.diagnostics_t,String,queue_size=10)
		self.node_status = rospy.Publisher("/node_status",String,queue_size=5)
		self.tilt = rospy.Publisher("/tilt",Int16,queue_size = 1)

		self.isOut_bound= False
		self.speed = 0.03

		self.previous_node= 1
		self.present_node = 2
		self.next_node= 0
		self.target_node=0

		self.collision_status=""

		self.dir=None

		self.isFirst = True
		self.isTurning=False
		self.p0=0
		self.p1=0

		#pid stuff
		self.kp = 2 #2
		self.ki = 0.03 #0.01
		self.kd = 0.01 #-0.05
		self.I = 0
		self.t = 0
		self.u = 0
		self.pid_=0
		self.prev_pid=0
		self.T= 1#0.000255
		self.e = 0
		self.D = 0
		self.temp = 0

		self.debug="0"


		#Variables path
		self.path=[]
		self.index=0

		#Barcode Var
		self.X = 0
	 	self.Y = 0

	def updatePresentState(self,msg):
		#print("Called Back ",msg.data[3])

		data=msg.data
		self.X=data[0]
		self.Y=data[1]
#		print ("X",self.X, "Y ",self.Y)

		#ang=data[2]*3.14/180.0
		#if(ang>3.14):
		#	ang=-3.14-(3.14-ang)
		if True:
			self.present_dir = data[2]
			self.present_node=msg.data[3]+1

			if self.present_node==self.next_node:
				if(self.index<len(self.path)):
					self.next_node=self.path[self.index]
					self.index+=1

#		self.diagnostics()
	#def align(self):

 	def stop(self):
		msg=geometry_msgs.msg.Twist()
		msg.linear.x = 0
		msg.linear.y = 0
		msg.angular.z = 0
		self.cmd_vel.publish(msg)

	def stop_linear(self):
		msg=geometry_msgs.msg.Twist()
		msg.linear.x = 0
		msg.angular.z = 0
		self.cmd_vel.publish(msg)

	def assignDir(self):
		d=utils.get_dir(self.present_node,self.next_node)
		if(d=="w"):
			self.target_dir = 175
			self.priority=1
		elif(d=="e"):
			self.target_dir = 355
			self.priority=1
		elif(d=="n"):
			self.target_dir = 85
			self.priority=0
		elif(d=="s"):
			self.target_dir = 265
			self.priority=0

		self.dir=d
		self.isFirst=False
		self.stop()
		self.isTurning = False
		rospy.sleep(0.1)

	def initialize_pid(self):
		self.I = 0
		self.t = 0
		self.u = 0
		self.e = 0
		self.D = 0
		self.temp = 0

	def pid(self):
		#if self.target_dir == 3.14 and self.present_dir < 0 :
		#	self.temp = -self.target_dir
		#	self.e = self.temp - self.present_dir
        	#else  :
		self.e = self.target_dir - self.present_dir
		if self.e<0:
			self.e=abs(self.e)
       		self.I = self.I + self.e*self.T
       		self.D = (self.e - self.t)/self.T
        	self.pid_ = self.kp*self.e + self.ki*self.I + self.kd*self.D
       		self.t = self.e
        	self.u = self.pid_-self.prev_pid
		self.prev_pid = self.pid_
		if abs(round(self.u*0.1,1))>0.1:
			self.u=-0.1
		else:
			self.u = 0.01

	def update(self):
		######Routine For First Time Initialisation#########
		if self.isFirst:
			time.sleep(0.2)
 			self.target_dir = self.present_dir
			self.dir="w"
			self.target_node=8
			self.entry= True
			self.path=[2,3,4,5,10,9,8]#pf.path(self.present_node,self.target_node)
			if(len(self.path)>0):
				self.next_node=self.path[0]
				self.index=1

		msg=geometry_msgs.msg.Twist()
		####Executes Turns###
		if abs(self.target_dir-self.present_dir) > 5 and not self.isFirst and (self.present_node == 5 or self.present_node == 10):
			self.isTurning = True
			self.debug="5"
			#self.pid()
			msg.angular.z= -0.08 #self.u
			msg.linear.x=0
			if abs(self.target_dir-self.present_dir) < 50:
				msg.angular.z= -0.05
		elif self.present_node==self.target_node:
			self.stop()
			msg2 = Int16()
			msg2.data = 1
			self.tilt.publish(msg2)
			exit()
			#self.target_node=utils.rand_valid_goal()
			#self.path=pf.path(self.present_node,self.target_node)

			#else:
			#	self.debug="6"
				#self.pid()
			#	msg.angular.z=-0.1 #self.u
			#	msg.linear.x=0
##		# New Direction/ Forward Movement #####
		else :
			msg.angular.z=0
			self.initialize_pid()
			if (self.present_node==self.next_node):
				print("FML")

			elif not(utils.get_dir(self.present_node,self.next_node)== self.dir):
				self.debug="9"
				self.assignDir()

			elif(utils.get_dir(self.present_node,self.next_node)==self.dir) :
				if self.isTurning:
					self.stop()
					rospy.sleep(0.1)

				self.isTurning=False				
				msg.linear.x=self.speed
				self.debug="8"

		self.cmd_vel.publish(msg)
	#	self.diagnostics()
	 	self.isFirst = False
	 	return 0

	def diagnostics(self):
		pass
		#msg = String()
		#msg.data = "present node: " + str(self.present_node) + " " +str(self.present_dir)+ " Debug" + self.debug  + "  " + "next node" + str(self.next_node) + "  present_dir: " +  str(self.present_dir) +  " " + " target_dir: " +  str(self.target_dir) +"  "+ "angular velocity:" + str(self.u) + "  Debug_Breakpoint " + self.debug #+ "Priority " + str(self.priority)
		##"  previous node" + str(self.previous_node) + "  target node" + str(self.target_node) +  
		#self.diagnostic.publish(msg)
