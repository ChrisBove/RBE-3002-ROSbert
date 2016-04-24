import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import math
import lab4 as lab4
#from rosbert_lab4.scripts.lab4.py import *

class aNode: 
	def __init__(self, index, val, huer, g): 
		self.index = index
		self.point = getWorldPointFromIndex(index)
		self.val = val 
		self.weight = val
		self.huer = huer 
		self.g = g 
		self.adjacent = list()
		self.f = 0
		self.cameFrom = -1
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
	unidentifiedCells = list()
	openCells = list()
	obstacles = list()


	unidentifiedCells = (cells for cells in mapData if cells == -1)	#cells that haven't been seen
	openCells = (cells for cells in mapData if cells <= 40 and cells >= -1)#cells that aren't obstacles
	obstacles = (cells for cells in mapData if cells > 40)			#cells that are obstacles

	for cells in openCells
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

#called after map topic is published.
#This fucntion goes to the closest unexplored area.
def captainKirk():




	lab4.publishObstacles(obstacles,resolution)

	return False


#I think this guy will just spin.
def scotty():

	return 0
	

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
	rospy.init_node('lab5')
	
	mapcomplete = False


	if mapData:
		lab4.initMap()
		while (not mapcomplete and not rospy.is_shutdown()):
			scotty()
			spock()
			mapcomplete = captainKirk()
			scotty()




	rospy.sleep(2)  
	print("Complete")







if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
