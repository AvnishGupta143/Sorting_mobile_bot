#!/usr/bin/env python
import rospy
import bot_3_pi as bot
import graph_lane_pi as graph
import networkx as nx
import time 


start=time.time()
count=0

class server(object):

	def __init__(self):
		self.num_bots=1
		self.bots=[]
		self.G=graph.G
		rospy.init_node("Central_Server",anonymous=True)
		self.count=0
		for i in range(self.num_bots):
			self.bots.append(bot.bot(i))
		self.p=0
								
	def update(self):
		
		for i in range(self.num_bots):
			self.count+=self.bots[i].update()


	
def exit():
	global Server
	Server.bots[0].stop()
	print("Exiting")

Server=server()	
rospy.on_shutdown(exit)
while not rospy.is_shutdown():
	Server.update()


		#rospy.spin()
