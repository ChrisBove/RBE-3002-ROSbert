""""
The A* search algorithm is an informed search algorithm of a known or explorable area, meaning that the program knows where the robot is, and where the goal is.  It also works with unexplored maps 
if the robot is equipped to explore and update the map in real time.  This algorithm works on any sort of graph that can be navigated, including directional graphs, but this lab uses a grid, so 
there are many ways to implement it.  Below is some modified pseudocode based on Wikipedia's article on A* (https://en.wikipedia.org/wiki/A*_search_algorithm) which I recommend reading to 
supplement course material.  The algorithm functions by expanding a frontier from the start node until it discovers the goal, in a method that could be described as a prioritized and incomplete 
depth-first search.  All nodes are assigned three values, the F, G, and H scores.  the G score of a node is the best navigable path that the search has found so far.  the H score is a heuristic 
that estimates the best possible distance between the node and the goal.  The F score is the sum of the G and H scores, and represents the best theoretical distance between the start node and the 
goal which travels through the current node.  By iterating the algorithm through the node on the frontier with the best F score, it will find the best possible path.

Authors: Wikipedia, Connor Flanigan
""" 

#it's recommended that start is a poseStamped msg and goal is a pose msg, RViz likes using that for visualization.
def AStar(start,goal):
    closedset = the empty set    # The set of nodes already evaluated.
    openset = [start]            # The set of tentative nodes to be evaluated, initially containing the start node.  The nodes in this set are the nodes that make the frontier between the closed 
	                             # set and all other nodes.
    came_from = the empty map    # The map of navigated nodes.
	
	# The g_score of a node is the distance of the shortest path from the start to the node.
	# Start by assuming that all nodes that have yet to be processed cannot be reached 
    g_score = map with default value of Infinity
	
	# The starting node has zero distance from start
    g_score[start] = 0
	
    # The f_score of a node is the estimated total cost from start to goal to the goal.  This is the sum of the g_score (shortest known path) and the h_score (best possible path).
    # assume same as g_score
	f_score = map with default value of Infinity  
	
	# heuristic_cost_estimate(a, b) is the shortest possible path between a and b, this can be euclidean, octodirectional, Manhattan, or something fancy based on how the machine moves
	# the best possible distance between the start and the goal will be the heuristic
    f_score[start] = g_score[start] + heuristic_cost_estimate(start, goal)
     
	
    while openset is not empty                                          # while there are still nodes that have not been checked, continually run the algorithm
	
        current = the node in openset having the lowest f_score[] value # this is the most promising node of all nodes in the open set
        if current = goal                                               # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            return reconstruct_path(came_from, goal)
         
        remove current from openset                  # mark this node as having been evaluated
        add current to closedset 
        for each neighbor in neighbor_nodes(current) # re-evaluate each neighboring node
            if neighbor in closedset
                continue
            tentative_g_score = g_score[current] + dist_between(current,neighbor) # create a new g_score for the current neighbor by adding the g_score from the current node and
			                                                                      # the distance to the neighbor
 
            if neighbor not in openset or tentative_g_score < g_score[neighbor]                 # if the neighbor has not been evaluated yet, or if a better path to the neighbor has been found,
				                                                                                # update the neighbor
                came_from[neighbor] = current                                                   # The node to reach this node from in the best time is the current node
                g_score[neighbor] = tentative_g_score                                           # The G score of the node is what we tentatively calculated earlier
                f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal) # The F score is the G score and the heuristic
                if neighbor not in openset                                                      # add this neighbor to the frontier if it was not in it already
                    add neighbor to openset
 
    return failure #if the program runs out of nodes to check before it finds the goal, then a solution does not exist

# Starting from the goal, work backwards to find the start.  We recommend returning a path nav_msgs, which is an array of PoseStamped with a header
def reconstruct_path(came_from,current):

	# start by adding goal to the path
    total_path = [current]
	
	# run while reconstruct_path hasn't reached the start
    while current in came_from:
		
		# The current node is now the node that leads to the previous node
        current = came_from[current]
		
		# add the current node to the front of the list
        total_path.append(current)
		
	# The list is now the shortest path from the start to the end
    return total_path
	
def heuristic_cost_estimate(start, goal):
	return #if there were no obstacles in the way of the robot, what is the shortest path to the goal?  Return that value
	
def neighbor_nodes(current):
	return #all nodes adjacent to the current node, this could be a list, an array, or any number of existing or custom data-types
	
def dist_between(current,neighbor):
	return #the distance necessary to travel to the neighbor from the current node
	
"""

some advice:

	A) don't be like me and make this monstrosity

"""
def generateEdges(self):

	y = 0
	while y < self.size[1]:
		x = 0
		while x < self.size[0]:
			j = y - 1
			while j <= y + 1:
				i = x - 1
				while i <= x + 1:
					if(conditions):
						if (more conditions):
							if (These many ifs were actually necessary without helper functions):
								if (this isn't a joke):
									if (don't let this be you):
										#code went here
					i += 1
				j += 1
			x += 1
		y += 1

	return
	
"""

	Remember, helper functions save lives

	B) Several groups used lists to store a node class, but traversing in a for loop and calling a method on each with a line that looks like list(i).method(args)
		By all logic, that should work, but in a lot of cases it seemed that all entries in the list had method(args) called on it.  It wasted a lot of time and effort,
		so look for that if, for instance, your A* tells you to go in a straight line from the start to the end.
	
"""