class Kirk(object):
	"""Boldly goes where no man has gone before"""
	def __init__(self):
		self.i = 0

	def boldlyGo(self, edges):
		""" Given list of edges, does all the things necessary to """
		
		# gets list of edges
		# runs through and calculates straighline lengths for all of them
		
		# chooses the one with the least cost - probably just straightline distance
			#in the future, we could run Astar on all of them and choose the one with best path
			# or have a history which picks the biggest one eventually
		# sends that as a goal to astar, lets robot move there and report it is done the move

	def spinAround(self):
		""" has robot spin around """

	def __saveEdges(self, edges):
		"""gets list of edges"""

	def __filterEdges(self):
		""" filters out the edges which are smaller than the robot
		 checks if we still have any left - otherwise, notify the makers"""


	def __pickClosestEdge(self):
		""" """

	def __sendGoalToAstar(self):


