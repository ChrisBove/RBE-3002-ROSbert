#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy as np
from numpy import dot
import math 
import copy
from tf.transformations import euler_from_quaternion


import heapq

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
    #global startIndex # don't make this global - we recalculate from robot pose now
	
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
    global goalTheta
    global goalOrientation

    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
	
    goalIndex = getIndexFromWorldPoint(goalX,goalY)
    print "Printing goal pose"
    print goal.pose
	
    goalOrientation = goal.pose.orientation
    quaternion = ( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    #convert yaw to degrees
    pose.orientation.z = yaw
    goalTheta = math.degrees(yaw)


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
	point.x=(getX(index)*resolution)+offsetX + (.5 * resolution)
	point.y=(getY(index)*resolution)+offsetY + (.5 * resolution)
	point.z=0
	return point

# returns the index number given a point in the world
def getIndexFromWorldPoint(x,y):
	#calculate the index coordinates
	indexX = int(((x-offsetX) - (.5*resolution))/resolution)
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
def findNeighbor(index):	 
	adjList = list() 

	currentPoint = Point()
	currentPoint.x = getX(index)
	currentPoint.y = getY(index)
	#print "I is %i, x is %i, y is %i" % (i, currentPoint.x, currentPoint.y)
	# try adding north
	if(isInMap(pointAbove(currentPoint))):	
		myPoint = pointAbove(currentPoint)
		#print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
		adjList.append(getIndexFromPoint(myPoint.x,myPoint.y))
	currentPoint.x = getX(index)
	currentPoint.y = getY(index)
	# try adding east
	if(isInMap(pointRight(currentPoint))):
		myPoint = pointRight(currentPoint)
		#print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
		adjList.append(getIndexFromPoint(myPoint.x,myPoint.y))
	currentPoint.x = getX(index)
	currentPoint.y = getY(index)
	# try adding south
	if(isInMap(pointBelow(currentPoint))):
		myPoint = pointBelow(currentPoint)
		#print "My Point X: %i Y: %i calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
		adjList.append(getIndexFromPoint(myPoint.x,myPoint.y))
	currentPoint.x = getX(index)
	currentPoint.y = getY(index)
	# try adding west
	if(isInMap(pointLeft(currentPoint))):
		myPoint = pointLeft(currentPoint)
		#print "My Point X: %i Y: %i  calc Index: %i" % (myPoint.x, myPoint.y,getIndexFromPoint(myPoint.x,myPoint.y))
		adjList.append(getIndexFromPoint(myPoint.x,myPoint.y))
	return adjList

#takes map data and converts it into nodes, calls linkMap function
def initMap(): 
	global frontier
	for i in range(0, width*height):

		node = aNode(i,mapData[i],heuristic(i),0.0, 0)
		G.append(node) 
		frontier.append(0)
	print len(G)	
	
	
def adjCellCheck(current):
	global adjList
	global traversal
	adjList =  findNeighbor(current.index) ## list of indexes of neighbor 
	for index in adjList:
		currCell = G[index] 
		if(currCell.val != 100):   #checks if cell is reachable  
			evalNeighbor(currCell, current) # evaluates the neighbor 
			traversal.append(G[index])
		if index == goalIndex:
			print "We found the goalllll!!!"
			break
	publishTraversal(traversal)
						

def evalNeighbor(nNode, current): 
	if(nNode not in closedSet):  # check if neighbor node is in closedSet - it has already been traveled to
		tentative = current.g + resolution  #checks what the potential cost to reach the node is 
		frontier.append(nNode)   
		publishFrontier(frontier)  # for rviz - publish node to frontier 
		if (nNode not in openSet) or (tentative < nNode.g):  # true if node has not already been added to frontier. or true if a previously established cost to reach the node is larger than the tentative cost to reach the node. 
			if (nNode not in openSet): # add nodes to openset 
				openSet.append(nNode)
			nNode.g = tentative # set cost to reach node 
			nNode.f = nNode.g + 1.5*nNode.huer # calc fScore 
			G[nNode.index].cameFrom = current.index  #set parent node - node traveled from to reach current node
	

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
	startIndex = getIndexFromWorldPoint(pose.position.x, pose.position.y)
	openSet.append(G[startIndex])        #Add first node to openSet # set priority to distance
	closedSet = list()		   #everything that has been examined
	
	print "start a*"
	
	print len(openSet)
	#print openSet[0].index

	while openSet:  # true when openSet (the frontier) is not empty, if open set is empty, that means no path can be found  

		try:  # exception handler so that you can cntrl^c the program  
			i = lowestInQ(openSet)     #find the node index/identifier of node in openSet with lowest travel cost 
			current = G[i]            
			if current in frontier:    # this is for graphical representation in rviz 
				frontier.remove(current)
			#print G[i].cameFrom
			if (current.index == goalIndex):         # found the destination 
				return reconPath(current, startIndex)
				pass 
			openSet.remove(current)                  #take node out of openset, the node is being explored - it's no longer part of frontier 
			closedSet.append(current)		 # add current node to closedSet (list of visited nodes) 
			adjCellList = adjCellCheck(current)      # look at neihboring nodes and add them to openset 
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


"""
rdp
~~~

Pure Python implementation of the Ramer-Douglas-Peucker algorithm.

:copyright: (c) 2014 Fabian Hirschmann <fabian@hirschmann.email>
:license: MIT, see LICENSE.txt for more details.

"""

def pldist(x0, x1, x2):
    """
    Calculates the distance from the point ``x0`` to the line given
    by the points ``x1`` and ``x2``.
    :param x0: a point
    :type x0: a 2x1 numpy array
    :param x1: a point of the line
    :type x1: 2x1 numpy array
    :param x2: another point of the line
    :type x2: 2x1 numpy array
    """
    if x1[0] == x2[0]:
        return np.abs(x0[0] - x1[0])

    return np.divide(np.linalg.norm(np.linalg.det([x2 - x1, x1 - x0])),
                     np.linalg.norm(x2 - x1))

def _rdp(M, epsilon, dist):
    """
    Simplifies a given array of points.

    :param M: an array
    :type M: Nx2 numpy array
    :param epsilon: epsilon in the rdp algorithm
    :type epsilon: float
    :param dist: distance function
    :type dist: function with signature ``f(x1, x2, x3)``
    """
    dmax = 0.0
    index = -1

    for i in xrange(1, M.shape[0]):
        d = dist(M[i], M[0], M[-1])

        if d > dmax:
            index = i
            dmax = d

    if dmax > epsilon:
        r1 = _rdp(M[:index + 1], epsilon, dist)
        r2 = _rdp(M[index:], epsilon, dist)

        return np.vstack((r1[:-1], r2))
    else:
        return np.vstack((M[0], M[-1]))


def _rdp_nn(seq, epsilon, dist):
    """
    Simplifies a given array of points.

    :param seq: a series of points
    :type seq: sequence of 2-tuples
    :param epsilon: epsilon in the rdp algorithm
    :type epsilon: float
    :param dist: distance function
    :type dist: function with signature ``f(x1, x2, x3)``
    """
    return rdp(np.array(seq), epsilon, dist).tolist()


def rdp(M, epsilon=0, dist=pldist):
    """
    Simplifies a given array of points.

    :param M: a series of points
    :type M: either a Nx2 numpy array or sequence of 2-tuples
    :param epsilon: epsilon in the rdp algorithm
    :type epsilon: float
    :param dist: distance function
    :type dist: function with signature ``f(x1, x2, x3)``
    """
    if "numpy" in str(type(M)):
        return _rdp(M, epsilon, dist)
    return _rdp_nn(M, epsilon, dist)

def getDouglasWaypoints(path):
	#convert path to numpy 2-d array
	a = np.zeros(shape = (len(path),2))
	for i,index in enumerate(path):
		point = Point()
		point = getWorldPointFromIndex(path[i])
		a[i] = [point.x, point.y]
	result = rdp(a,epsilon=resolution*2)
	#turn numpy 2-d array back into some form of path (list of points)
	resultList = list()
	for x in range(result.size/2):
		point = Point()
		point.x = result[x,0]
		point.y = result[x,1]
		point.z = 0
		resultList.append(point)
	return resultList
	#return DouglasPeucker(path, epsilon)		


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

#keeps track of current location and orientation
def tCallback(event):
    global pose
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)

def statusCallback(status):
    global moveDone
    moveDone = status

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

    global pose
    global odom_list
    #global odom_tf
    pose = Pose()
    global goalTheta
    global theta

    global moveDone
    moveDone = Bool()


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

    move_pub = rospy.Publisher('clicked_pose', PoseStamped, None, queue_size=1)
    move_status_sub = rospy.Subscriber('/moves_done', Bool, statusCallback, queue_size=1)

    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
    
    odom_list = tf.TransformListener() #listner for robot location

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(2)

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        if goalRead:
            moveDone = False
            path = aStar()
            print "Going to publish path"
            publishPath(noFilter(path))
            print "Publishing waypoints"
            waypoints = getDouglasWaypoints(path)
            publishWaypoints(waypoints)#publish waypoints
            print "Finished... beginning robot movements"
            #for each waypoint
            for waypt in waypoints:
                print "doing a new waypoint:"
                print waypt
                # if this is the last waypoint, instead take the goal orientation
                if (abs(goalX - waypt.x) <= resolution) and (abs(goalY - waypt.y) <= resolution):
                    orientation = goalOrientation
                #calculate end orientation for waypoint - perhaps the angle to the next one? or just our current heading?
                else:
                    orientation = pose.orientation
                #publish goal to topic to move the robot
                wayPose = PoseStamped()
                wayPose.pose.position.x = waypt.x
                wayPose.pose.position.y = waypt.y
                wayPose.pose.position.z = 0
                wayPose.pose.orientation = orientation
                move_pub.publish(wayPose)
                #wait for robot to arrive at waypoint (should be a service?)
                errorDist = math.sqrt(pow(wayPose.pose.position.x - pose.position.x,2)+pow(wayPose.pose.position.y - pose.position.y,2))
                errorTheta = goalTheta - theta 
                print "errorDist: %f errorTheta: %f" % (errorDist, errorTheta)

                while (not rospy.is_shutdown()) and not moveDone :
                    #chill out. Drink some coffee
                    errorDist = math.sqrt(pow(wayPose.pose.position.x - pose.position.x,2)+pow(wayPose.pose.position.y - pose.position.y,2))
                    errorTheta = goalTheta - theta 
                    rospy.sleep(0.5)
                    #print "errorDist: %f errorTheta: %f" % (errorDist, errorTheta)
                moveDone = False
                

            goalRead = False
        rospy.sleep(2)  
        #print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
