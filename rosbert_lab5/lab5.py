import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import math
from lab4 import *
#from rosbert_lab4.scripts.lab4.py import *



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


#this finds doors/frontiers
def boldlyGo():
	unidentifiedCells = list()
	openCells = list()
	obstacles = list()

	unidentifiedCells = (node for node in mapData if node.val -1)
	openCells = (node for node in mapData if node.val <= 40 and node.val > -1)
	obstacles = (node for node in mapData if node.val > 40)


	

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





#Main handler of the project
def run():
<<<<<<< HEAD
	global mapData

	map_sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	rospy.init_node('lab5')
	while (1 and not rospy.is_shutdown()):
		if mapData:
=======

    map_sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)



	rospy.init_node('lab5')


	while (1 and not rospy.is_shutdown()):
		if mapData
>>>>>>> origin/master
			boldlyGo()


	rospy.sleep(2)  
	print("Complete")







if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass