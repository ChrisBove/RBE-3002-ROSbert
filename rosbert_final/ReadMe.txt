Code Overview - Operation:

	Subscribe to ros Nodes: 
		/map 
		/spin_done
		/nav_done
		/nav_failed 
	
	Create publishers: 
		/map_cells/obstacles
		goal_pose
		spin_me


	Scotty() - the spin command 
	Spock(nodeList) - takes a list of nodes, finds frontiers 
	captainKirk() - Check for valid frontiers, if frontiers exist choose best one 
			- find centroid of frontier, send destination to A* (in lab4_updated) 
			- Send A* waypoints to navigation in lab2

To run start following nodes in terminal
On turtlebot: 
	roslaunch turtlebot_bringup minimal.launch --screen
	roslaunch turtlebot_navigation gmapping_demo.launch
	rosrun rosbert_final lab2.py 

On ROS Master Computer: 
rosrun rviz rviz  
	Change rviz configuration to a*config.rviz
 rosrun rosbert_final lab5.py
 rosrun rosbert_final lab4.py 

See: http://wiki.ros.org/turtlebot/Tutorials/indigo for instructions of turtlebot & pc network setup 
  