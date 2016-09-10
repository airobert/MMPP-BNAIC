# Robert White
# Gepetto team, Robotics Department, LAAS-CNRS, France
# ILLC, University of Amsterdam, The Netherlands
# ai.robert.wangshuai@gmail.com


import copy

# this is a class to represent the searching tree
# because so far there is no case that all agents got blocked,
# so I didn't implement the backtrack part

class Node ():
	# configs = []
	agent_remained = []
	paths = []
	children = []
	absolute_time = 0
	progress_time = 0


	def __init__(self, configs):
		self.agent_remained = range (len(configs)) # all the agents
		self.paths = map ((lambda x: [x]), configs)

	def cloneNode (self):
		node = Node ([])
		node.paths = copy.copy(self.paths)
		node.agent_remained = copy.copy(self.agent_remained)
		self.absolute_time = copy.copy(self.absolute_time)
		self.progress_time = copy.copy(self.progress_time)
		return node

	def expand (self, indexes_and_paths, reached_agents):
		# print '\n \n \n  these agents are removed!!!!!!!!!', reached_agents
		child = self.cloneNode()
		for (index, path) in indexes_and_paths:
			if child.paths[index][-1] == path[0]:
			# if True:
				child.paths[index] = child.paths[index] + path[1::]
				print '\n We CAN continue planning from here\n' 
			else:
				print index, ' --- ',reached_agents
				print '\nERROR: can not continue from here\n\n'
				print 'should be: ', child.paths[index][-1][0], child.paths[index][-1][1]
				print 'but is it', path[0][0], path[0][1]

		child.progress_time = len (indexes_and_paths[0]) -1
		child.absolute_time = self.absolute_time + child.progress_time
		child.agent_remained = self.agent_remained
		for a in reached_agents:
			child.agent_remained.remove(a)
		return child

	def terminates(self):
		return (self.agent_remained == [])

	def getAgentCurrentConfig(self, agent):
		return self.paths[agent][-1]

	def getAgentsRemained(self):
		return self.agent_remained

	def printInformation(self):
		print 'absolute time: ', self.absolute_time
		print 'progress_time: ', self.progress_time
		print '---------------------------'

	def getAgentPlan (self, agent):
		return self.paths[agent]
