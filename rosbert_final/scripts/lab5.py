#!/usr/bin/env python 

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import math
import lab4_updated as lab4
from lab4_updated import aNode
#from rosbert_lab4.scripts.lab4.py import *

# class aNode: 
# 	def __init__(self, index, val, huer, g): 
# 		self.index = index
# 		self.point = getWorldPointFromIndex(index)
# 		self.val = val 
# 		self.weight = val
# 		self.huer = huer 
# 		self.g = g 
# 		self.adjacent = list()
# 		self.f = 0
# 		self.cameFrom = -1
# 	def addParent(self, index): 
# 		self.cameFrom = (index)

# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info


#takes map data and converts it into nodes, calls linkMap function
def initMap(): 
	print "creating map" 
	global frontier
	for i in range(0, width*height):
		node = aNode(i,mapData[i],heuristic(i),0.0)
		G.append(node) 

	print "map created" 

#finds the frontiers on the map. puts them in a list of objects?
#a two dimensional list of border nodes. maybe a dictionary?
def spock():
	global edgelist
	global frontier

	frontier = list

	unidentifiedCells = list()
	openCells = list()
	obstacles = list()
	edgelist = list()

	unidentifiedCells = (cells for cells in mapData if cells == -1)	#cells that haven't been seen
	openCells = (cells for cells in mapData if cells <= 40 and cells >= -1)#cells that aren't obstacles
	obstacles = (cells for cells in mapData if cells > 40)			#cells that are obstacles


	#if a cell has neighbors in the unidentified zone, it is a frontier
	print "Finding frontier"
	for cell in openCells:
		for neighbor in lab4.findNeighbors(cell):
			if neighbor in unidentifiedCells:
				frontier.add(cell)

	print "finding edges"
	#if a node hasn't been added to an edge yet, add all of its attached nodes to a new list
	for cell in frontier:
		if not listCheck2D(cell, edgelist):
			edgelist.add(findedge(cell,list()))
 
 	publishFrontier(edgelist)
 
#recursive strategy for travelling along edges to enumerate the frontier groups
def findedge(cell,list):
	#For neighbors of cells in the frontier
	if cell not in list:
		list.add(cell)
	for neighbor in lab4.findNeighbors(cell):
		if neighbor in frontier:
			if neighbor not in list:
				#add it to the list of the edge
				list.add(neighbor)
				#find its connected neighbors and add them to the list in a similar manner
				findedge(neighbor,list)

	return list


#simply checks through a list of lists
def listCheck2D(cell, multilist):
	for list in multilist:
		if cell in list:
			return True
		else:
			return False


#called after map topic is published.
#This fucntion goes to the closest unexplored area.
def captainKirk():




	lab4.publishObstacles(obstacles,resolution)

	return True


#I think this guy will just spin.
def scotty():

	return 0
	

def publishFrontier(grid):
    global pub_frontier
        # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for node in grid:
        point=Point()
        point = getWorldPointFromIndex(node.index)
        cells.cells.append(point)
    pub_frontier.publish(cells) 


def publishCells(grid):
	global pub	
	#print "publishing"

    # resolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution

	for i in range(1,height): #height should be set to height of grid
		k=k+1
		for j in range(1,width): #width should be set to width of grid
			k=k+1
            #print k # used for debugging
			if (grid[k] == 100):
				point=Point()
				point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
				point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
				point.z=0
				cells.cells.append(point)
	pub.publish(cells)           





#Main handler of the project
#this function looks around, 
#calls boldly go, 
#looks around again to see if we can call boldly go again
def run():
	global mapData
	global width
	global height
	map_sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)

	pub_frontier = rospy.Publisher('map_cells/frontier', GridCells, queue_size=1)



	rospy.init_node('lab5')
	
	mapcomplete = False


	if mapData:
		lab4.initMap(width, height,mapData)
		while (not mapcomplete and not rospy.is_shutdown()):
			scotty()
			spock()
			mapcomplete = captainKirk()
			scotty()




	rospy.sleep(2)  
	print("Complete")







if __name__ == '__main__':
    try:
    	#lab4.run()
        run()
    except rospy.ROSInterruptException:
        pass
