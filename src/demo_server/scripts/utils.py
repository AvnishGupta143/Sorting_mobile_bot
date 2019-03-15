#!/usr/bin/env python

#File depends on node_max
import random
import numpy as np
import graph_lane_pi as graph

g =graph.G

valid_goals=[i for i in range(1,9)]
node_max=5
def rand_valid_goal():
	return random.choice(valid_goals)
		
def xy(node):
	x=(node/node_max)+1
	y= node%node_max
	if(y==0): 
		y=node_max
		x=x-1
	return (x,y)

def node_xy(x,y):
	return (x-1)*node_max+y


def get_dir(present_node,next_node):
	try:
		d=g[present_node][next_node]["dir"]
		return d
	except:
		return "/"


