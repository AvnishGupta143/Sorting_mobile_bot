#import graph100 as graph
import graph_lane_pi as graph
import heapq
import numpy as np
import utils
node_max= utils.node_max # multiple unit that we are using


class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def xy(node):
	x=(node/node_max)+1
	y= node%node_max
	if(y==0): 
		y=node_max
		x=x-1
	return (x,y)

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return np.sqrt(np.square(x1 - x2) + np.square(y1 - y2))


def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.successors(current):
            new_cost = cost_so_far[current] + 1  # add edge cost
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(xy(goal), xy(next))
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

def path(start,goal):
	#print(start , " --> ", goal)
	came_from, cost_so_far = a_star_search(g,start,goal)
	#print("completed")

	current = goal 
	path = []
	while current != start: 
   		path.append(current)
   		current = came_from[current]

	#path.append(start) 
	path.reverse() 

	return path

g=graph.G


