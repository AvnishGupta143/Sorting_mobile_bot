#!/usr/bin/env python
import networkx as nx
import matplotlib.pyplot as plt

G= nx.DiGraph()

length = 80
width  = 60

for i in range(1,10):
	G.add_node(i)

for i in range(1,5):
	G.add_edge(i,i+1,dir="w")

for i in range(6,10):
	G.add_edge(i+1,i,dir="e")

G.add_edge(1,6,dir="s")
G.add_edge(7,2,dir="n")
G.add_edge(3,8,dir="s")
G.add_edge(9,4,dir="n")
G.add_edge(5,10,dir="s")
###print("12")

#A = nx.nx_agraph.to_agraph(G)
#A.layout()
#A.draw('layout_demo.png')
print("Done")

