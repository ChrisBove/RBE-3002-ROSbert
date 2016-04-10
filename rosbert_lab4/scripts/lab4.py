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
#needed for Douglas Peucker algorithm
import visual
import itertools

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
		self.cameFrom = -1
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
	h = math.sqrt(pow(goalX-current.x,2)+pow(goalY-current.y,2))
	return h

def findConnected(node):

	neighborhood = G.neighbors(node)
	#print "Printing neighborhood"
	#for node in neighborhood:
		#frontier[node] = 100
	#publishFrontier(frontier)
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
			nNode.g = calcG(current.g+nNode.g, nNode.g)
			nNode.f = nNode.g + 2*nNode.huer 
			G[nNode.index].cameFrom = current.index
	#is in the closed set
	else:
		lowestInQ(openSet)

#shitty sort finds lowest cost node 
def lowestInQ(nodeSet): 
	costList = list() 
	for node in nodeSet:
		costList.append(node.f)

	a = costList.index(min(costList))
	mapIndex = nodeSet[a].index
	return mapIndex
	
def reconPath(current, start): 
	total_path = list()
	total_path.append(current.index)
	 		
	while (current.cameFrom != -1):
		current = G[current.cameFrom]
		total_path.append(current.cameFrom)		
			 
	return total_path
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
			if current in frontier: 
				frontier.remove(current)
			#print G[i].cameFrom
			if (current.index == goalIndex): 
				print reconPath(current, G[startIndex])
								
				return reconPath(current, startIndex)
				pass
			openSet.remove(current)
			closedSet.append(current)		
			adjCellList = adjCellCheck(current)
			if adjCellList:
				for node in adjCellList:
					if node not in closedSet:
						frontier.append(node)
						publishFrontier(frontier)
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
	for i,node in enumerate(path):
		averagePoint = Point()
		if(i+1 < len(path)):
			nextNode = path[i+1]
			nextNodeX = getWorldPointFromIndex(nextNode).x
			nextNodeY = getWorldPointFromIndex(nextNode).y

			currNode = path[i]
			currNodeX = getWorldPointFromIndex(currNode).x
			currNodeY = getWorldPointFromIndex(currNode).y
			if( not returnPath):
				averagePoint.x = (currNodeX+nextNodeX)/2
				averagePoint.y = (currNodeY+nextNodeY)/2
				averagePoint.z = 0#math.atan2(currNodeY-nextNodeY, currNodeX-nextNodeX)
			else:
				averagePoint.x = (returnPath[i-1].x+currNodeX)/2
				averagePoint.y = (returnPath[i-1].y+currNodeY)/2
				averagePoint.z = 0 #math.atan2(currNodeY-returnPath[i-1].y, currNodeX-returnPath[i-1].x)
			returnPath.append(averagePoint)
			#print "Average Point in Path: X: %f Y: %f" % (averagePoint.x, averagePoint.y)
	return returnPath

def smoothPathPoints(path): #takes the parsed path & tries to remove unecessary zigzags 
	returnPath = list()
	for i,node in enumerate(path):
		averagePoint = Point()
		if(i+1 < len(path)):
			nextNode = path[i+1]
			nextNodeX = nextNode.x
			nextNodeY = nextNode.y

			currNode = path[i]
			currNodeX = currNode.x
			currNodeY = currNode.y
			if( not returnPath):
				averagePoint.x = (currNodeX+nextNodeX)/2
				averagePoint.y = (currNodeY+nextNodeY)/2
				averagePoint.z = 0#math.atan2(currNodeY-nextNodeY, currNodeX-nextNodeX)
			else:
				averagePoint.x = (returnPath[i-1].x+currNodeX)/2
				averagePoint.y = (returnPath[i-1].y+currNodeY)/2
				averagePoint.z = 0 #math.atan2(currNodeY-returnPath[i-1].y, currNodeX-returnPath[i-1].x)
			returnPath.append(averagePoint)
			#print "Average Point in Path: X: %f Y: %f" % (averagePoint.x, averagePoint.y)
	return returnPath

def noFilter(path): #takes the parsed path & tries to remove unecessary zigzags 
	returnPath = list()
	for i,node in enumerate(path):
		point = Point()
		currNode = path[i]
		point.x = getWorldPointFromIndex(currNode).x
		point.y = getWorldPointFromIndex(currNode).y
		point.z = 0
		
		returnPath.append(point)
		#print "Point in Path: X: %f Y: %f" % (point.x, point.y)
	return returnPath

def distance(point1, point2):
	return math.sqrt(pow(point2.x-point1.x,2)+pow(point2.y-point1.y,2))

#calculates the perpendicular distance between a line and a point
# http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
def perpendicularDistance(point, linePoint1, linePoint2):
	#                     p         v           w
	# Return minimum distance between line segment vw and point p
	l2 =  pow(linePoint2 - linePoint2, 2) #length_squared(v, w) i.e. |w-v|^2 -  avoid a sqrt
	if (l2 == 0.0): 
		return distance(point, linePoint1);   # v == w case
	# Consider the line extending the segment, parameterized as v + t (w - v).
	# We find projection of point p onto the line. 
	# It falls where t = [(p-v) . (w-v)] / |w-v|^2
	# We clamp t from [0,1] to handle points outside the segment vw.
	t = max(0, min(1, dot(point - linePoint1, linePoint2 - linePoint1) / l2));
	projection = linePoint1 + t * (linePoint2 - linePoint1);  # Projection falls on the segment
	return distance(point, projection); 

#runs Douglas Peucker algorithm on passed list to reduce points of path into waypoints
# see https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
def DouglasPeucker(path, epsilon):
	dmax = 0
	index = 0
	end = len(path)

	#save first point
	firstPoint = Point()
	fistNode = path[0]
	firstPoint.x = getWorldPointFromIndex(currNode).x
	firstPoint.y = getWorldPointFromIndex(currNode).y
	firstPoint.z = 0
	#save last point
	lastPoint = Point()
	lastNode = path[end-1]
	lastPoint.x = getWorldPointFromIndex(currNode).x
	lastPoint.y = getWorldPointFromIndex(currNode).y
	lastPoint.z = 0

	for i in range(2, end):
		currPoint = Point()
		currNode = path[i]
		currPoint.x = getWorldPointFromIndex(currNode).x
		currPoint.y = getWorldPointFromIndex(currNode).y
		currPoint.z = 0
		
		d = perpendicularDistance(currPoint, firstpoint, lastPoint)
		
		if (d > dmax):
			index = i
			dmax = d

	#if max distance is greater than epsilon, simplify recursively
	if  (dmax > epsilon):
		#create list from 0 to index
		#create list from index to end-1

		recRes1 = DouglasPeucker(itertools.islice(path, 0 , i), epsilon)
		recRes2 = DouglasPeucker(itertools.islice(path, i , end-1), epsilon)

		#build the restul list
		resultList = list()
		resultList.append(itertools.islice(recRes1, 0, len(recRes1)-1))
		resultList.append(itertools.islice(recRes2, 0, len(recRes2))
	else:
		resultList = list()
		resultList.append(path, 0, end-1)
	return resultList

def getDouglasWaypoints(path)
	epsilon = 0.5
	return DouglasPeucker(path, epsilon)		

#this picks out linear positions along the path
def getWaypoints(path):
	returnPath = list()
	point = Point()
	pointNode = path[0]
	point.x = getWorldPointFromIndex(pointNode).x
	point.y = getWorldPointFromIndex(pointNode).y
	point.z = 0
	returnPath.append(point)
	for i,node in enumerate(path):
		currPoint = Point()
		currNode = path[i]
		currPoint.x = getWorldPointFromIndex(currNode).x
		currPoint.y = getWorldPointFromIndex(currNode).y
		currPoint.z = 0

		if (i+1 < len(path)):
			nextPoint = Point()
			nextNode = path[i+1]
			nextPoint.x = getWorldPointFromIndex(nextNode).x
			nextPoint.y = getWorldPointFromIndex(nextNode).y
			nextPoint.z = 0

			if(math.degrees(math.fabs(math.atan2(nextPoint.y-currPoint.y,nextPoint.x-nextPoint.y))) >= 10):
				returnPath.append(currPoint)
		else:
			returnPath.append(currPoint)
			pass

		#print "Point in Path: X: %f Y: %f" % (point.x, point.y)
	return returnPath

#publishes map to rviz using gridcells type

def publishCells(grid):
	global pub
	#print "publishing"

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

def publishPath(grid):
    global pub_path
    #print "publishing traversal"

        # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for node in grid:
        point=Point()
        point = node
        cells.cells.append(point)
	#print "Point in Path: X: %f Y: %f" % (point.x, point.y)
    pub_path.publish(cells)  

def publishWaypoints(grid):
    global pubway
    #print "publishing traversal"

        # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for node in grid:
        point=Point()
        point = node
        cells.cells.append(point)
	#print "Point in Waypoint: X: %f Y: %f" % (point.x, point.y)
    pubway.publish(cells) 

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
    global pub_path
    global pubway
    global frontier
    frontier = list()
    global goal_pub




    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pub_path = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
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
            path = aStar()
            print "Going to publish path"
            publishPath(noFilter(path))
            print "Publishing waypoints"
            publishWaypoints(getDouglasWaypoints(path))#publish waypoints
            print "Finished..."
            goalRead = False
        rospy.sleep(2)  
        #print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
