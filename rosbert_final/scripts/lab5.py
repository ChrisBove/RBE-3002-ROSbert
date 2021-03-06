#!/usr/bin/env python 

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
from tf.transformations import euler_from_quaternion
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
    global nodeList
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    mapReceived = True
    lab4.mapCallBack(data)
    nodeList = initMap(data)
    print data.info

def isInMapXY(x, y):
	return True
	#catch if point is negative
	if(x < 0 or y < 0):
		return False
	# is point within 0 and width and 0 and height?
	if( ( 0 <= x and width > x) and ( 0 <= y and height > y)):
		return True
	else:
		print "not in map"
		return False
def obstacleExpansion(G):
	global pub_obs
	print "expanding nodes"
	numberOfNodesExpanded = 0
	robotSize = .25
	obstacles = list()
	map_obs = list()
	#map_obs = (node for node in G if node.val > 30)
	for node in G:
		if node.val > 92:
			map_obs.append(node)
	for obsNode in map_obs:
		obsx = obsNode.point.x
		obsy = obsNode.point.y

		for distance in range(0, 5):# math.trunc(robotSize/resolution)):
			try:
				if(isInMapXY(obsx + distance*resolution, obsy)):
					eastindex = getIndexFromWorldPoint(obsx + distance*resolution, obsy)
					east = G[eastindex]
					if(east.weight < obsNode.val):
						east.weight = obsNode.val
					obstacles.append(east)
				if(isInMapXY(obsx - distance*resolution, obsy)):
					westindex = getIndexFromWorldPoint(obsx - distance*resolution, obsy)
					west = G[westindex]
					if(west.weight < obsNode.val):
						west.weight = obsNode.val
					obstacles.append(west)
				if(isInMapXY(obsx,obsy + distance*resolution)):
					northindex =  getIndexFromWorldPoint(obsx,obsy + distance*resolution)
					north = G[northindex]
					if(north.weight < obsNode.val):
						north.weight = obsNode.val
					obstacles.append(north)
				if(isInMapXY(obsx,obsy - distance*resolution)):
					southindex =  getIndexFromWorldPoint(obsx,obsy - distance*resolution)
					south = G[southindex]
					if(south.weight < obsNode.val):
						south.weight = obsNode.val
					obstacles.append(south)
					numberOfNodesExpanded = numberOfNodesExpanded + 1

				if(isInMapXY(obsx+distance*resolution,obsy + distance*resolution)):
					northeastindex = getIndexFromWorldPoint(obsx+distance*resolution,obsy + distance*resolution)
					northeast = G[northeastindex]
					if(northeast.weight < obsNode.val):
						northeast.weight = obsNode.val
					obstacles.append(northeast)
					numberOfNodesExpanded = numberOfNodesExpanded + 1
				if(isInMapXY(obsx-distance*resolution,obsy + distance*resolution)):
					northwestindex = getIndexFromWorldPoint(obsx-distance*resolution,obsy + distance*resolution)
					northwest = G[northwestindex]
					if(northwest.weight < obsNode.val):
						northwest.weight = obsNode.val
					obstacles.append(northwest)
					numberOfNodesExpanded = numberOfNodesExpanded + 1
				if(isInMapXY(obsx+distance*resolution,obsy - distance*resolution)):
					southeastindex = getIndexFromWorldPoint(obsx+distance*resolution,obsy - distance*resolution)
					southeast = G[southeastindex]
					if(southeast.weight < obsNode.val):
						southeast.weight = obsNode.val
					obstacles.append(southeast)
					numberOfNodesExpanded = numberOfNodesExpanded + 1
				if(isInMapXY(obsx-distance*resolution,obsy - distance*resolution)):
					southwestindex = getIndexFromWorldPoint(obsx-distance*resolution,obsy - distance*resolution)
					southwest = G[southwestindex]
					if(southwest.weight < obsNode.val):
						southwest.weight = obsNode.val
					obstacles.append(southwest)
					numberOfNodesExpanded = numberOfNodesExpanded + 1

			except IndexError:
				pass

	publishObstacles(obstacles)
	print "=================Num obs: %d Num found: %d" %(len(map_obs),numberOfNodesExpanded)
	return G



def publishObstacles(grid):
	global pub_obs
    #print "publishing traversal"

        # mapresolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution

	for node in grid:
		point=Point()
		point = node.point
		cells.cells.append(point)
	pub_obs.publish(cells)  


#takes map data and converts it into nodes, calls linkMap function
def initMap(mapData): 
	print "creating map" 
	#global frontier
	G = list()
	for i in range(0, width*height):
		node = aNode(i,mapData.data[i],0,0.0)
		G.append(node) 
	obstacleExpansion(G)

	return G
	print "map created" 

#finds the frontiers on the map. puts them in a list of objects?
#a two dimensional list of border nodes. maybe a dictionary?
def spock(G):
	global edgelist
	global frontier

	frontier = list()

	#unidentifiedCells = list()
	openCells = list()
	obstacles = list()
	edgelist = list()
	frontiernodes = list()
	
	# for cell in G:
	# 	if cell.weight == -1: 
	# 		unidentifiedCells.append(cell)	#cells that haven't been seen
	for cell in G:
		if cell.weight <= 80 and cell.weight >= 0:#cells that aren't obstacles:
			openCells.append(cell)
	for cell in G:
		if cell.weight > 80:
			obstacles.append(cell) 			#cells that are obstacles

	#print "Num unidentified: %d" % len(unidentifiedCells)
	print "Num open: %d" % len(openCells)

	#if a cell has neighbors in the unidentified zone, it is a frontier
	print "Finding frontier"
	for cell in openCells:
		for neighborindex in lab4.findNeighbor(cell.index,True):
			if G[neighborindex].weight == -1 and G[neighborindex] not in frontier:#if the neighbor is unidentified
				frontier.append(cell)
				#print "append to frontier"

	print "frontier size: %d" % len(frontier) 

	print "finding edges"
	#if a node hasn't been appended to an edge yet, append all of its attached nodes to a new list
	for cell in frontier:
		if not listCheck2D(cell, edgelist):
			edge = findedge(cell,list(),G)
			publishFrontier(edge)
			edgelist.append(edge)
			print "=================  edge #: %d, size: %d" % (len(edgelist),len(edge))

 	for edge in edgelist:
 		for node in edge:
 			frontiernodes.append(node)
	publishFrontier(frontiernodes)
 
#recursive strategy for travelling along edges to enumerate the frontier groups
def findedge(cell,edge,G):
	#For neighbors of cells in the frontier
	if cell not in edge:
		edge.append(cell)
	for neighborindex in lab4.findNeighbor(cell.index,True):#return nodes that are neighbors
		if G[neighborindex] in frontier and G[neighborindex] not in edge:
			#append it to the edge of the edge
			edge.append(G[neighborindex])
			#print "node: %d" % neighborindex
			#find its connected neighborindexs and append them to the edge in a similar manner
			findedge(G[neighborindex],edge,G)

	return edge


#simply checks through a list of lists
def listCheck2D(cell, multilist):
	for list in multilist:
		if cell in list:
			return True
	else:
		return False
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
	return index



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

#called after map topic is published.
#This fucntion goes to the closest unexplored area.
# @return True if we have completed the map
def captainKirk():
	"""Boldly goes where no man has gone before"""
	distances = list() # list of float distances
	centroids = list() # list of np 2-d arrays [x,y] world coordinates
	minDistance = 99999
	closestEdge = -1
	centroidIndex = 0


	# runs through and calculates straighline lengths for all of them
	for i,edge in enumerate(edgelist):
		# calculates straighline lengths for all of them
		start = edge[0].point
		end = edge[len(edge)-1].point
		width = math.sqrt(pow(end.x-start.x,2)+pow(end.y-start.y,2))
		print "edge %i, width %d" % (i, width)
		print "startX %d, startY %d" % (start.x, start.y)
		print "end X %d, end Y %d" % (end.x, end.y)
		
		# filters out the edges which are smaller than the robot
		#if width <= 0.3556:
		if len(edge) < (0.3556/(resolution))/2.0:
			#TODO need to add another filter to try running an astar path to that point
			edgelist.remove(edge)
			if len(edgelist) == 0:

				break
		else:
			# we assume all edges are concave away from robot - otherwise we could pick unknown space
			
			#TODO Instead of just going to center of the straighline, we could run rdp on it
			# first, and then find the middle waypoint. That would fix the curve issue
			thing = int(math.ceil(len(edge)/2))
			point = Point()
			point.x = edge[thing].point.x
			point.y = edge[thing].point.y


			#calculate vector between start and end of edge
			# vector = np.array([(end.x-start.x) + end.x, (end.y-start.y) + end.y])
			# normalized = vector/np.linalg.norm(vector)

			# #calculate the x,y along that vector that gets us the midpoint
			# centroid = (0.5*width)*normalized
			# centroid[0] += start.x
			# centroid[1] += start.y
			# point = Point()
			# point.x = centroid[0]
			# point.y = centroid[1]

			robotX = pose.position.x
			robotY = pose.position.y

			# calculate straightline distance to the center of this edge from robot
			distance = math.sqrt(pow(point.x-robotX,2)+pow(point.y-robotY,2))
			print "Distance: %f minDistance: %f" % (distance, minDistance)

			#check if this is the shortest distance
			if distance < minDistance:
				minDistance = distance #update min distance
				closestEdge = centroidIndex # save which edge is best
				centroidIndex += centroidIndex
			distances.append(distance) # so the distances correspond to index of edgelist
			centroids.append(point)
	#TODO catch if we didn't find a closest edge index

	print "The closest edge index: %i" % closestEdge
	# checks if we still have any left - otherwise, notify the makers
	while len(edgelist) > 0 and closestEdge != -1:
		# chooses the one with the least cost - probably just straightline distance
			#in the future, we could run Astar on all of them and choose the one with best path
			# or have a history which picks the biggest one eventually
		orientation = pose.orientation
		#publish goal to topic to move the robot
		wayPose = PoseStamped()
		wayPose.pose.position.x = centroids[closestEdge].x
		wayPose.pose.position.y = centroids[closestEdge].y
		wayPose.pose.position.z = 0
		wayPose.pose.orientation = orientation

		global spinDone, navFailed
		spinDone = False
		global navCallbackServiced
		navCallbackServiced = False
		goalPub.publish(wayPose)
		# sends that as a goal to astar, lets robot move there and report it is done the move
		print "waiting for astar path"
		waitForValidPath() # waits until navFailed is set by callback
		if navFailed:
			print "Our nav failed..."
			edgelist.remove(edgelist[closestEdge])

			# check list again and pick closest one
			distances = list() # list of float distances
			centroids = list() # list of np 2-d arrays [x,y] world coordinates
			minDistance = 99999
			closestEdge = -1
			centroidIndex = 0
			# runs through and calculates straighline lengths for all of them
			for i,edge in enumerate(edgelist):
				# calculates straighline lengths for all of them
				start = edge[0].point
				end = edge[len(edge)-1].point
				width = math.sqrt(pow(end.x-start.x,2)+pow(end.y-start.y,2))
				print "edge %i, width %d" % (i, width)
				print "startX %d, startY %d" % (start.x, start.y)
				print "end X %d, end Y %d" % (end.x, end.y)
				
				# filters out the edges which are smaller than the robot
				#if width <= 0.3556:
				if len(edge) < 0.3556/(resolution):
					#TODO need to add another filter to try running an astar path to that point
					edgelist.remove(edge)
				else:
					# we assume all edges are concave away from robot - otherwise we could pick unknown space
					
					#TODO Instead of just going to center of the straighline, we could run rdp on it
					# first, and then find the middle waypoint. That would fix the curve issue
					thing = int(math.ceil(len(edge)/2))
					point = Point()
					point.x = edge[thing].point.x
					point.y = edge[thing].point.y


					#calculate vector between start and end of edge
					# vector = np.array([(end.x-start.x) + end.x, (end.y-start.y) + end.y])
					# normalized = vector/np.linalg.norm(vector)

					# #calculate the x,y along that vector that gets us the midpoint
					# centroid = (0.5*width)*normalized
					# centroid[0] += start.x
					# centroid[1] += start.y
					# point = Point()
					# point.x = centroid[0]
					# point.y = centroid[1]

					robotX = pose.position.x
					robotY = pose.position.y

					# calculate straightline distance to the center of this edge from robot
					distance = math.sqrt(pow(point.x-robotX,2)+pow(point.y-robotY,2))

					#check if this is the shortest distance
					if distance < minDistance:
						minDistance = distance #update min distance
						closestEdge = centroidIndex # save which edge is best
						centroidIndex += centroidIndex
					distances.append(distance) # so the distances correspond to index of edgelist
					centroids.append(point)

			continue
		else:
			print "waiting for robot to move"
			navDone = False
			waitForRobotToMove()

			return False
	
	
	print "No more valid edges"
	return True

def waitForRobotToMove():
	global navDone
	navDone = False
	while (not rospy.is_shutdown() and not navDone):
		pass
	print "Robot has finished moving"
	navDone = False

def waitForValidPath():
	global navCallbackServiced
	while (not rospy.is_shutdown() and not navCallbackServiced):
		pass
	navCallbackServiced = False

def waitForRobotToSpin():
	global spinDone
	while (not rospy.is_shutdown() and not spinDone):
		pass
	spinDone = False

#I think this guy will just spin.
def scotty():
	global spinDone
	spinDone = False
	spin_pub.publish(True)

	print "Waiting for robot to finish spinning"
	waitForRobotToSpin()
	print "Spinning complete"
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
        point = lab4.getWorldPointFromIndex(node.index)
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

#keeps track of current location and orientation
def tCallback(event):
    global pose
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)

def spinStatusCallback(status):
    print "spinStatusCallback: %r" % status
    global spinDone
    spinDone = True

def navStatusCallback(status):
    """THis fires when the navigation is done."""
    print "navStatusCallback: %r" % status
    global navDone
    navDone = True

def navFailedCallback(status):
    """ This fires whenever we finish trying a a star path """
    print "navFailedCallback: %r" % status
    global navFailed, navCallbackServiced
    navFailed = status.data
    navCallbackServiced = True




#Main handler of the project
#this function looks around, 
#calls boldly go, 
#looks around again to see if we can call boldly go again
def run():
	longstring = '''\
	                             _  ___________________________=====____
              __--__                / ||     =<==== NCC-1701 =======       /
___________---______---___________  \_||__________________________________/
\________________________________/           | |
             \______/  \__ .. :  \           | |
               `--'       \_   :  \          | |
                         __-`------`-----____| |
                      \ |||_     .::. :      |_|--_
                      -)=|__ =<=======--      :. |_|
                      / |||           __________---'
                          ------------
	'''
	print longstring
	rospy.init_node('lab5')
	global nodeList
	global mapData
	global mapgrid
	global width
	width = 0
	global height
	global pub_frontier
	map_sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)#rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, mapCallBack)

	global pub_obs
	pub_frontier = rospy.Publisher('map_cells/frontier', GridCells, queue_size=1)
	pub_obs = rospy.Publisher('/map_cells/obstacles', GridCells, queue_size=1)

	global goalPub #publishing the goal to astar
	goalPub = rospy.Publisher('goal_pose', PoseStamped, queue_size=1)

	global pose
	global odom_list
	pose = Pose()
	rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
	odom_list = tf.TransformListener() #listner for robot location

	global spin_pub
	spin_pub = rospy.Publisher('spin_me', Bool, queue_size=1)

	# this is for spinning
	global spinDone
	spinDone = False
	move_status_sub = rospy.Subscriber('/spin_done', Bool, spinStatusCallback)

	global navDone
	global navCallbackServiced
	navCallbackServiced = False
	navDone = False
	nav_status_sub = rospy.Subscriber('nav_done', Bool, navStatusCallback)
	nav_failed_sub = rospy.Subscriber('nav_failed', Bool, navFailedCallback)
	# wait a second for publisher, subscribers, and TF
	rospy.sleep(2)
	
	mapcomplete = False


	while not width and not rospy.is_shutdown():
		rospy.sleep(0.1)
		pass

	while (not mapcomplete and not rospy.is_shutdown()):
		scotty()
		scotty()
		#map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, mapCallBack)
		#G = initMap(mapgrid)#lab4.initMap(mapgrid)
		spock(nodeList)
		mapcomplete = captainKirk()
		#scotty()




	rospy.sleep(2)  
	print("Complete")

	print longstring





if __name__ == '__main__':
    try:
    	#lab4.run()
        run()
    except rospy.ROSInterruptException:
        pass
