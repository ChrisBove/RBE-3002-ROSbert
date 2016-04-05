#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math
import networkx as nx

nx
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

def readStart(_startPos):
    global startPosX
    global startPosY
    global startPos
    startPos = _startPos
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print "Printing start pose"
    print startPos.pose.pose

def readGoal(goal):
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print "Printing goal pose"
    print goal.pose
    aStar(startPos,goal)

def heuristic(current, goal): 
	dx = abs(current.x - goal.x) 
	dy = abs(current.y - goal.y) 
	h = (dx+dy)*.01             #tie breaker
   	return h

def nodeToX(node): #not sure if this is needed - find x coordinate of a node
	#TODO
	pass 
def nodeToY(node): # not sure if this is needed - find y coordinate of a node 
	#TODO 
	pass 
def xyToNode(x, y): #I think this is needed to convert start pose (x,y,z) to a node that is in the map 
	#TODO
	pass 

def linkMap(G):	 
	for i in range(1, G.size()):
		print (test)
		# add node -> (next) 
 		# as long as node is not last in row and the one next to it
		if ((i % width) > 0):  
			G.add_edge(i+1)
		
		currentRow = height / i
		# add node ^ (up) 
		if (i < height): 
			G.add_edge(i+width) 
		# add node / (up to right) 		
		if ((i<height) & ((i % width) > 0)):
			G.add_edge(i+width + 1) 			 
	return G

def initMap(): 
	G = nx.Graph()
	for i in range(1, width*height): 
		G.add_node(i,weight = mapData[i])
		
	print(G.size()) ## THIS SHOULD NOT BE 0
	G = linkMap(G)
	
	
		


def gScore(path,current): 
	#TODO
	pass 

	
def checkIsShortestPath (something):
	#TODO
	pass 

def adjCellCheck(cellList):
	for i in range(1, len(cellList)):
		currCell = cellList.index(i)
		if(currCell != 100):  
			if( currCell not in closeSet): 
				if(currCell not in openSet): 
					openSet.add(currCell) 
			## unfinished A* stuff... 

def aStar(start,goal):
	global path 
	global openSet
	global closedSet
	path = list()          # the path
	openSet = PriorityQueue()  #frontier - unexplored 
	openSet.put(start,0)        
	closedSet = set()		   #everything that has been examined
	gScore = list() 
	fScore = list()  
	gScore[start] = 0								
	fScore[start] = gScore[start] + heuristic(start, goal) 	#cost so far
	
#	while not openSet.empty():  
#		current = openSet.get()
#		closeSet.add(current)
#		if current == goal: 
#			return path
#
#		else:
#			adjCellList = getAdj(current)
# 			if not adjCellList.empty()
#				
			
			
############################################# 
#            print G.number_of_nodes()

#    for i in range(1,height*width):
#        if mapData[i] == 0: 
#           G.add_node(i,weight = mapData[i])
#            print G.number_of_nodes()
    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points

def parsePath(path):  #takes A* path, output the nodes where the path changes directions  
	#TODO
	pass 
def smoothPaht(path): #takes the parsed path & tries to remove unecessary zigzags 
	#TODO
	pass


#publishes map to rviz using gridcells type

def publishCells(grid):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(1,height): #height should be set to hieght of grid
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
def run():
    global pub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('goal_pose', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('start_pose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)


    initMap() 
    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
