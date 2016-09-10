# gepetto-viewer-server
# not hpp-manipulation-server
# hppcorbaserver
# -DCMAKE_INSTALL_PREFIX=/home/airobert/HPP/install

from Environment import BasicHouse
from Obstacle import Obstacle
from hpp.corbaserver import ProblemSolver
# from hpp.corbaserver.manipulation import ProblemSolver as MProblemSolver, ConstraintGraph
from hpp.gepetto import PathPlayer
from hpp.gepetto import ViewerFactory
# from hpp.gepetto.manipulation import ViewerFactory as MViewerFactory
# from hpp.corbaserver.manipulation.pr2 import Robot
# from hpp.corbaserver.manipulation import robot as METARobot
# from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from Node import Node
from time import sleep

class Platform ():
	# main_agent = None
	agents = []
	# problem solver
	ps = None
	# path player
	pp = None
	# view factory
	vf = None
	# viewer
	r = None
	env = None
	# a dictionary to get the agent's index
	index_dic = {}

	#for tree searching
	tree = None # the root of the three
	current_node = None

	# pp = PathPlayer (rbprmBuilder.client.basic,ls r)
	def __init__(self, agents):
		self.agents = agents
		for i in range (len(agents)):
			a = agents[i]
			self.index_dic[agents[i].robot.name] = i
			self.agents[i].registerPlatform(self, i)
			print 'the agent ', agents[i].robot.name, ' is now registered with the index ', self.getInidex(agents[i].robot.name) 
		self.tree = self.getStartNode()
		self.current_node = self.tree

	def getStartNode(self):
		init_configs = []
		for a in self.agents:
			init_configs.append(a.start_config)
		return Node(init_configs)

	def start(self):
		# self.problem.selectProblem(0)
		self.ps = ProblemSolver(self.agents[0].robot)
		self.vf = ViewerFactory(self.ps)
		if self.env != None:
			self.vf.loadObstacleModel(self.env.packageName, self.env.urdfName, self.env.name)
		self.r = self.vf.createViewer()
		for a in self.agents:
			a.startDefaultSolver()
			a.setBounds()
			a.setEnvironment()
			a.solve()
			a.storePath()
			# self.loadAgentView(a.index)
			# self.r(a.start_config)
			print 'the agent ', a.robot.name, ' now has a backup plan of length', a.getProposedPlanLength()


		# self.pp = PathPlayer (self.agents[0], self.r)


	def loadAgentView (self, index, default = False): #default position or not
		self.ps = self.agents[index -1].ps
		self.vf = ViewerFactory (self.ps)
		# self.vf.loadObstacleModel(self.env.packageName, self.env.urdfName, self.env.name)
		self.r = self.vf.createViewer()
		# print '---------------->', len(self.agents[index - 1].init_config)
		if default:
			self.r(self.agents[index - 1].current_config)
		# self.r.computeObjectPosition()

	def getInidex(self, robot_name):
		return self.index_dic[robot_name]


	def setEnvironment(self, env):
		self.env = env
		# self.ps.moveObstacle('airbase_link_0', [0,0, -3, 1,0,0,0])
		# self.r = self.vf.createViewer()

	# def startViewer(self):
	# 	self.r = vf.createViewer()

	def updateViewAtTime(self, t):
		config = []
		for a in self.agents:
			config.append (a.getConfigOfProposedPlanAtTime(t))
		self.r(config)


	def playAllProposedPath(self):
		print 'play proposed path'
		max_time = 0
		for a in self.agents:
			l = a.getProposedPlanLength()
			if l > max_time:
				max_time = l
		
		for t in range(max_time):
			# print 'time is ', t
			for i in range(len(self.agents)):
				a = self.agents[i]
				if  a.getProposedPlanLength() > t:
					# print 'agent ', a.index, 
					self.loadAgentView(i+1)
					# and then set the agent to its current configuration
					self.r(a.getConfigOfProposedPlanAtTime(t))
			# sleep(0.003)

	def playAllPermittedPath(self):
		max_time = 0
		for a in self.agents:
			l = a.getPermittedPlanLength()
			if l > max_time:
				max_time = l
		
		for t in range(max_time):
			# print 'time is ', t
			for i in range(len(self.agents)):
				a = self.agents[i]
				if  a.getPermittedPlanLength() > t:
					# print 'agent ', a.index, 
					self.loadAgentView(i+1)
					# and then set the agent to its current configuration
					self.r(a.getConfigOfPermittedPlanAtTime(t))
			# sleep(0.003)


	def validateAllPaths(self, agents_remained):
		print '******* start validation **********'
		# print agents_remained

		max_time = 0
		for i in agents_remained:
			a = self.agents[i]

			a.startDefaultSolver()
			a.setBounds()
			a.setEnvironment()
			a.loadOtherAgents()

			l = a.getProposedPlanLength()
			if l > max_time:
				max_time = l

		for t in range (max_time):
			print '\n this is time ', t
			for i in agents_remained:
				a = self.agents[i]

				a.startDefaultSolver()
				a.setBounds()
				a.setEnvironment()
				a.loadOtherAgents()

				# print 'this is robot ', a.robot.name
				# a1.obstacle.getObstacleNames(False, 1000)
				if a.getProposedPlanLength() > t:
					# myconfig = a.getConfigOfProposedPlanAtTime(t)
					# myspec = a.getMoveSpecification(myconfig)
					# print 'the agent is at ', myspec[0], myspec[1]
					# first of all, move all the obstacles
					for oa in self.agents: # other agents
						if a.index != oa.index:
							# print '\t and moving the ghost of ', oa.robot.name
							if not (oa.index in agents_remained) or oa.getProposedPlanLength() <= t:
								config = oa.end_config
							else:
								config = oa.getConfigOfProposedPlanAtTime(t)
							spec = oa.getMoveSpecification(config)
							a.obstacle.moveObstacle(oa.robot.name + 'base_link_0', spec)
							print '\tmove ghost', oa.robot.name, ' to ', spec[0], spec[1] 

					# secondly, test if the configuration is valid 
					(result, _) = a.robot.isConfigValid(a.getConfigOfProposedPlanAtTime(t))
					if not result:
						return t
		# if everything is fine at each time slot, return -1 
		return -1 



	def construct_tree (self, iteration):
		print '******************* this is iternation ', iteration, ' ***********************'
		self.current_node.printInformation()
		if iteration > 0:
			#expand the tree by doing planning for each agent and find the collision momment
			for i in self.current_node.getAgentsRemained():
				a = self.agents[i]
				print '>>>>>>>>>>>>this is agent', a.robot.name , ' computing ' 
				if (a.computePlan(self.current_node) == -1):
					print 'the agent is now using its backup plan/plan from last time'
					# line = input()

			# self.playAllProposedPath()

			t = self.validateAllPaths(self.current_node.getAgentsRemained())
			print 'in this iteration, the collision appears at time ', t
			if t == -1: # the path is valid! we terminate the process
				paths = []
				indexes_and_paths = []
				for i in self.current_node.agent_remained:
					a = self.agents[i]
					indexes_and_paths.append((a.index, a.obtainProposedPlan()))

				child = self.current_node.expand(indexes_and_paths, [])
				return (True, child.paths)
			else:
				reached = []
				indexes_and_paths = []
				for i in self.current_node.getAgentsRemained():
					a = self.agents[i]
					if a.getProposedPlanLength() > t - 1: # up to t, because t is the moment of collision!
						path = a.obtainProposedPlan()[:t-1]
						indexes_and_paths.append((i, path))
					else:
						reached.append(i) # reached, therefore remove from the remaining list
						indexes_and_paths.append((i, a.obtainProposedPlan()))
				self.current_node = self.current_node.expand(indexes_and_paths, reached) 
				
				return self.construct_tree(iteration - 1)
		else: # can not find a path for each agent within limited iteration
			return (False, None)


# remove those who has already got to where they suppose to be
# at least one step ----------- frangment