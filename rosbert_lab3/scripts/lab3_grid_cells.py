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
import copy

import heapq
nx

## implementation from Python Cookbook
class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

    def push(self, item, priority):
        heapq.heappush(self._queue, (priority, self._index, item))
        self._index += 1

    def pop(self):
        return heapq.heappop(self._queue)[-1]
	
class aNode: 
	def __init__(self, index, val, huer, g, adjacent): 
		self.index = index
		self.val = val 
		self.huer = huer 
		self.g = g 
		self.adjacent = list()
		self.f = 0
		self.cameFrom = list()
	def addAdjacent(self, index):
		self.adjacent.append(index)
	def addParent(self, index): 
		self.cameFrom = (index)

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
    global startRead
    startRead = True
    global startPosX
    global startPosY
    global startPos
    global startIndex
	
    startPos = _startPos
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    
    startIndex = getIndexFromWorldPoint(startPosX, startPosY)
    print "Printing start pose"
    print startPos.pose.pose
    point = getWorldPointFromIndex(startIndex)
    print "Calculated world position: %f, %f Index: %i" % (point.x, point.y, startIndex)

def readGoal(goal):
    global goalRead
    goalRead = True
    global goalX
    global goalY
    global goalIndex
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
	
    goalIndex = getIndexFromWorldPoint(goalX,goalY)
    print "Printing goal pose"
    print goal.pose

#returns in meters the point of the current index
def getPointFromIndex(index):
	point=Point()
	point.x=getX(index)
	point.y=getY(index)
	point.z=0
	return point

# returns the index number given a point in the world
def getIndexFromPoint(x,y):
	return int(((y)*width) + x)

#returns in meters the point of the current index
def getWorldPointFromIndex(index):
	point=Point()
	#print "GetX: %i" % getX(index)
	point.x=(getX(index)*resolution)+offsetX + (1.5 * resolution)
	point.y=(getY(index)*resolution)+offsetY + (.5 * resolution)
	point.z=0
	return point

# returns the index number given a point in the world
def getIndexFromWorldPoint(x,y):
	#calculate the index coordinates
	indexX = int(((x-offsetX) - (1.5*resolution))/resolution)
	indexY = int(((y-offsetY) - (.5*resolution))/resolution)
	
	index = int (((indexY)*width) + indexX) 
	
	print index	
	return index

def heuristic(index): 
	current = getWorldPointFromIndex(index)
	# calc manhattan distance
	dx = abs(current.x - goalX) 
	dy = abs(current.y - goalY) 
	h = (dx+dy)
	#print "I is %i h is %f" % (index,h)
	return h


def findConnected(node):

	neighborhood = G.neighbors(node)
	print "Printing neighborhood"
	for node in neighborhood:
		frontier[node] = 100
	publishFrontier(frontier)
	return neighborhood



#returns the x value of the index
def getX(index):
	adjusted = index + 1
	if (adjusted % width) == 0:
		return width - 1
	else:
		return (adjusted % width) - 1

#returns the y value of the index
def getY(index):
	adjusted = index
	return math.floor(adjusted/width)
	
#checks if the passed point is in the map
def isInMap(point):
	#catch if point is negative
	if(point.x < 0 or point.y < 0):
		return False
	# is point within 0 and width and 0 and height?
	if( ( 0 <= point.x and width > point.x) and ( 0 <= point.y and height > point.y)):
		return True
	else:
		return False

#returns index of point above this one, only works for non-first row
def pointAbove(point):
	output = copy.deepcopy(point)
	output.y += 1
	return output

#returns index of point below this one, only works for non-last row
def pointBelow(point):
	output = copy.deepcopy(point)
	output.y -= 1
	return output

#returns index of point right of this one, only works for non-last column
def pointRight(point):
	output = copy.deepcopy(point)
	output.x += 1
	return output

#returns index of point right of this one, only works for non-first column
def pointLeft(point):
	output = copy.deepcopy(point)
	output.x -= 1
	return output

#this adds the edges to the graphs
def linkMap():	 
	for i in range(0, height*width):
		currentPoint = Point()
		currentPoint.x = getX(i)
		currentPoint.y = getY(i)
		#print "I is %i, x is %i, y is %i" % (i, currentPoint.x, currentPoint.y)
		# try adding north
		if(isInMap(pointAbove(currentPoint))):	
			myPoint = pointAbove(currentPoint)
			#print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
			G[i].addAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))
		currentPoint.x = getX(i)
		currentPoint.y = getY(i)
		# try adding east
		if(isInMap(pointRight(currentPoint))):
			myPoint = pointRight(currentPoint)
			#print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
			G[i].addAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))
		currentPoint.x = getX(i)
		currentPoint.y = getY(i)
		# try adding south
		if(isInMap(pointBelow(currentPoint))):
			myPoint = pointBelow(currentPoint)
			#print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
			G[i].addAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))
		currentPoint.x = getX(i)
		currentPoint.y = getY(i)
		# try adding west
		if(isInMap(pointLeft(currentPoint))):
			myPoint = pointLeft(currentPoint)
			#print "My Point X: %i Y: %i  calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
			G[i].addAdjacent(getIndexFromPoint(myPoint.x,myPoint.y))
	#for i in range(0, height*width):
		#print "I is %i" % i
		#print G[i].adjacent

#takes map data and converts it into nodes, calls linkMap function
def initMap(): 
	global frontier
	for i in range(0, width*height):

		node = aNode(i,mapData[i],heuristic(i),0.0, 0)
		G.append(node) 
		frontier.append(0)
	print len(G)	
	linkMap()
	
#check's and/or compute's cell's g-score based on current g-score
def gScore(cumulativeScore,index): 
	#TODO
	pass 

	
def checkIsShortestPath (something):
	#TODO
	pass 

def calcG(currentG, neighborG):
	if (neighborG == 0): 
		neighborG = currentG + resolution
	return neighborG
	
	
def adjCellCheck(current):
	global adjList
	global traversal
	adjList =  current.adjacent ## list of indexes of neighbor 
	for index in adjList:
		currCell = G[index] 
		if(currCell.val != 100): 
			evalNeighbor(currCell, current) 
		traversal.append(G[index])
		if index == goalIndex:
			print "We found the goalllll!!!"
			break
	publishTraversal(traversal)
						

def evalNeighbor(nNode, current): 
	if(nNode not in closedSet): 
		tentative = current.g + resolution 
		if (nNode not in openSet) or (tentative < nNode.g): 
			if (nNode not in openSet):
				openSet.append(nNode)
			nNode.g = calcG(current.g, nNode.g)
			nNode.f = nNode.g + nNode.huer 
			cameFromList = (current.cameFrom)
			cameFromList.append(current.index)
			G[nNode.index].addParent(cameFromList)
	#is in the closed set
	else:
		lowestInQ(openSet)

#shitty sort finds lowest cost node 
def lowestInQ(nodeSet): 
	costList = list() 
	for node in nodeSet:
		costList.append(node.huer + node.g)

	a = costList.index(min(costList))
	mapIndex = nodeSet[a].index
	return mapIndex
	

def aStar():
	
	global G
	G = list()
	initMap()  # add all nodes to grah, link all nodes

	global path 
	path = list()
	global openSet
	global closedSet

	global traversal
	traversal = list()
	global frontier
	frontier = list()

	openSet = list()
	openSet.append(G[startIndex])        #Add first node to openSet # set priority to distance
	closedSet = list()		   #everything that has been examined
	
	print "start a*"
	
	print len(openSet)
	#print openSet[0].index

	while openSet:  

		try:
			i = lowestInQ(openSet) 
			current = G[i]
			#print G[i].cameFrom
			if (current.index == goalIndex): 
				return current.cameFrom
				pass
			openSet.remove(current)
			closedSet.append(current)		
			adjCellList = adjCellCheck(current)
		except KeyboardInterrupt: 
			break
 	
	print "No route to goal"
			



    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points
	G.clear()

def parsePath(path):  #takes A* path, output the nodes where the path changes directions  
	#TODO
	pass 
def smoothPath(path): #takes the parsed path & tries to remove unecessary zigzags 
	#TODO

	returnPath = list()
	averagePoint = Point()
	for i,node in enumerate(path):
		nextNode = path[index+1]
		nextNodeX = getPointFromIndex(nextNode.index).x
		nextNodeY = getPointFromIndex(nextNode.index).y

		currNode = path[index]
		currNodeX = getPointFromIndex(currNode.index).x
		currNodeY = getPointFromIndex(currNode.index).y
		if( not returnPath):
			averagePoint.x = (currNodeX+nextNodeX)/2
			averagePoint.y = (currNodeY+nextNodeY)/2
			averagePoint.z = atan2(currNodeY-nextNodeY, currNodeX-nextNodeX)
		else:
			averagePoint.x = (returnPath[i-1].x+currNodeX)/2
			averagePoint.y = (returnPath[i-1].y+currNodeY)/2
			averagePoint.z = atan2(currNodeY-returnPath[i-1].y, currNodeX-returnPath[i-1].x)
		returnPath.append(averagePoint)

	return returnPath



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


def publishFrontier(grid):
    global pub_frontier
    print "publishing frontier"

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




def publishTraversal(grid):
    global pub_traverse
    #print "publishing traversal"

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
    pub_traverse.publish(cells)  

def pubGoal(grid):
	global goal_pub
	
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution
	cells.cell_height = resolution

	for node in grid:
		point = getWorldPointFromIndex(node.index)
		cells.cells.append(point)
	goal_pub.publish(cells)


#Main handler of the project
def run():

    global pub
    global startRead
    global goalRead
    startRead = False
    goalRead = False
    global pub_frontier
    global pub_traverse
    global frontier
    frontier = list()
    global goal_pub




    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('goal_pose', PoseStamped, readGoal, queue_size=1) #change topic for best results
    pub_traverse = rospy.Publisher('map_cells/traversal', GridCells, queue_size=1)
    pub_frontier = rospy.Publisher('map_cells/frontier', GridCells, queue_size=1)
    start_sub = rospy.Subscriber('start_pose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
    goal_pub = rospy.Publisher('goal_point', PoseStamped, queue_size=1)
    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        if startRead and goalRead:
            aStar()
            goalRead = False
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
