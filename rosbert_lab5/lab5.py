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

#called after map topic is published.
#This fucntion goes to the closest unexplored area.

def boldlyGo():
	unidentifiedCells = list()
	openCells = list()
	obstacles = list()

	unidentifiedCells = (cells for cells in mapData if cells == -1)	#cells that haven't been seen
	openCells = (cells for cells in mapData if cells <= 40 and cells >= -1)#cells that aren't obstacles
	obstacles = (cells for cells in mapData if cells > 40)			#cells that are obstacles

	lab4.publishObstacles(obstacles,resolution)

	return False


#I think this guy will just spin.
def lookAround():

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
			lookAround()
			mapcomplete = boldlyGo()
			lookAround()



	rospy.sleep(2)  
	print("Complete")







if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass