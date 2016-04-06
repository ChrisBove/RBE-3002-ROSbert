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
    startPos = _startPos
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print "Printing start pose"
    print startPos.pose.pose

def readGoal(goal):
    global goalRead
    goalRead = True
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print "Printing goal pose"
    print goal.pose

#returns in meters the point of the current index
def getPointFromIndex(index):
	i = getX(index)
	j = getY(index)
	point=Point()
	point.x=(j*resolution)+offsetX + (1.5 * resolution)
	point.y=(i*resolution)+offsetY - (.5 * resolution)
	point.z=0
	return point

def heuristic(index): 
	current = getPointFromIndex(index)
	# calc manhattan distance
	dx = abs(current.x - goalX) 
	dy = abs(current.y - goalY) 
	h = (dx+dy)
	return h

def setPriority(g, h): 
	return g+h

def nodeToXY(node): #not sure if this is needed - find x and y coordinates of a node
	#TODO
	pass  
def xyToNode(x, y): #I think this is needed to convert start pose (x,y,z) to a node that is in the map 
	#TODO
	pass 


def findConnected(node):
    neighborhood = G.neighbors(node)
    print (len(neighborhood))
    #print "Printing neighborhood"
    for node in neighborhood:
        frontier[node] = 100
        print node 
    publishFrontier(frontier)
    return neighborhood


#returns the x value of the index
def getX(index):
	if (index % width) == 0:
		return width
	else:
		return (index % width)

#returns the y value of the index
def getY(index):
	return math.ceil(index/width)
	
#checks if the passed point is in the map
def isInMap(point):
	#catch if point is negative
	if(point < 0):
		return False
	# is point within 1 and width and 1 and height?
	if( ( 1 <= getX(point) and width >= getX(point)) and ( 1 <= getY(point) and height >= getY(point))):
		return True
	else:
		return False

#returns index of point above this one, only works for non-first row
def indexAbove(index):
	return index - width

#returns index of point below this one, only works for non-last row
def indexBelow(index):
	return index + width

#returns index of point right of this one, only works for non-last column
def indexRight(index):
	return index + 1

#returns index of point right of this one, only works for non-first column
def indexLeft(index):
	return index - 1

#this adds the edges to the graphs
def linkMap():	 
	for i in range(1, height*width):

		# try adding north
		if(isInMap(i)):
			G.add_edge(i, indexAbove(i))
		# try adding east
		if(isInMap(i)):
			G.add_edge(i, indexRight(i))
		# try adding south
		if(isInMap(i)):
			G.add_edge(i, indexBelow(i))
		# try adding west
		if(isInMap(i)):
			G.add_edge(i, indexLeft(i))

#takes map data and converts it into nodes, calls linkMap function

def initMap(): 
	global frontier
	for i in range(1, width*height):
		G.add_node(i,value = mapData[i],h=heuristic(i),g=0.0)
		frontier.append(0)
	linkMap()
	#for node in G: 
	#	findConnected(node)

#check's and/or compute's cell's g-score based on current g-score
def gScore(cumulativeScore,index): 
	#TODO
	pass 

	
def checkIsShortestPath (something):
	#TODO
	pass 

def adjCellCheck(current):
	global adjList
	adjList = list()
	adjList = findConnected(current) 
	print(adjList)
	for i in range(1, len(adjList)):
		currCell = adjList.index(i)
		if(currCell != 100): 
			print (currCell) 
			##if(currCell not in closeSet): 
			#	if(currCell not in openSet): 
			#		openSet.put(currCell) 
			#	#else if( check shortest path) 
					#calc G + h
		else:
			 break
			## unfinished A* stuff... 
			## findConnected(node)

def aStar():
	global G
	G = nx.Graph()
	initMap()  # add all nodes to grah, link all nodes

	global path 
	global openSet
	global closedSet
	path = nx.Graph()
	start = 0
	path.add_node(start)
	openSet = PriorityQueue()  #frontier - unexplored 
	openSet.push(start,0)        # set priority to distance
	openSet.push(5,1)
	closedSet = set()		   #everything that has been examined
	gScore = list() 
	fScore = list()  
#	gScore[start] = 0								
#	fScore[start] = gScore[start] + heuristic(start, goal) 	#cost so far
	goal = 500
	
	print "start a*"
	
	while openSet:  
		current = openSet.pop()
		closedSet.add(current)
		if current == goal: 
			return path
		else:
			print "looking at adj" 
			adjCellList = adjCellCheck(current)
			print "done looking at adj"
 		#	if not adjCellList.empty()
	pass		
			
	print "end a*"
			
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
	G.clear()

def parsePath(path):  #takes A* path, output the nodes where the path changes directions  
	#TODO
	pass 
def smoothPath(path): #takes the parsed path & tries to remove unecessary zigzags 
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


def publishFrontier(grid):
    global pub_frontier
    print "publishing frontier"

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
    pub_frontier.publish(cells)  


class amjNode: 
	def __init(self, index, val, huer, g): 
		self.index = index
		self.val = val 
		self.huer = huer 
		self.g = g 
		self.adj = list()
	def addAdj(index):
		self.adj.append(index)



#Main handler of the project
def run():

#    new = amjNode(1, 100, 5, 0)
#    new.addAdj(5)
	
#    print (new.adj)


    global pub

    global startRead
    global goalRead
    startRead = False
    goalRead = False
    global pub_frontier
    global frontier
    frontier = list()




    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('goal_pose', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('start_pose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
    pub_frontier = rospy.Publisher("/grid_cells/frontier",GridCells,queue_size=10)
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
