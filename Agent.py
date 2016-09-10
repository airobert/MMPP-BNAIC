# Robert White
# Gepetto team, Robotics Department, LAAS-CNRS, France
# ILLC, University of Amsterdam, The Netherlands
# ai.robert.wangshuai@gmail.com

#import from HPP
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
from hpp import Error
from hpp.gepetto import ViewerFactory
#import other assistant functions
import sys
from math import cos, sin, asin, acos, atan2, pi
from time import sleep
import copy
from  threading import Timer
#import other classes for better mangement of classes and simulation 
from Ghost import Ghost
from HyQ import HyQ

# this class is main Agent class. It consists of a robot, a problem,
# some obstacles (the environment and other agents). It's also a manager of problems and 
# an interface to communicate and interact with the platform

class Agent (Client):
	robot = None
	platform = None
	index = 0
	ps = None
	# we load other agents as ghosts to reduce the computation time while planning
	ghosts = []
	ghost_urdf = ''
	ghost_package = ''
	# to avoid confusion, we use start and end instead of init and goal
	start_config = [] 
	end_config = []
	current_config = [] # this is not used for now
	permitted_plan = [] # this is the plan generated
	repeat = 0 # to help the selectProblem function

	# an agent should have a robot and the start and end configuration
	# to avoid confusion, we use start_config instead of init_config  and
	# end_confi instead of goal_config
	def __init__ (self, robot, start, end):
		Client.__init__ (self)
		self.repeat = 0
		# print 'creating an agent of type ', robotType 
		self.robot = robot
		self.start_config = start
		self.end_config = end
		self.current_config = self.start_config
		self.__plan_proposed = []


	# once all agents are generated, we may register the agents to a platform
	def registerPlatform(self, platform, index):
		self.platform = platform
		self.index = index

	# this function gives some information about the agent and robot it is managing
	def printInformation(self):
		print '-------------------------------------------'
		print 'name of the robot:\t', self.robot.name
		print 'configuration size:\t', self.robot.getConfigSize()
		print 'degree of freedom:\t', self.robot.getNumberDof()
		print 'mass of the robot:\t', self.robot.getMass()
		print 'the center of mass:\t', self.robot.getCenterOfMass()
		config = self.robot.getCurrentConfig()
		nm = self.robot.getJointNames()
		print 'there are ', len(nm), 'joint names in total. They are:'
		for i in range(len(nm)):
			lower = self.robot.getJointBounds(nm[i])[0]
			upper = self.robot.getJointBounds(nm[i])[1]
			print 'joint name: ', nm[i], '\trank in configuration:', self.robot.rankInConfiguration[nm[i]],
			print '\tlower bound: {0:.3f}'.format(lower), '\tupper bound: {0:.3f}'.format(upper) 
		
	# set up the environment
	def setEnvironment(self):
		if self.platform.env != None:
			self.ps.loadObstacleFromUrdf(self.platform.env.packageName, self.platform.env.urdfName, self.platform.env.name)
			# self.ps.moveObstacle('airbase_link_0', [0,0, -3, 1,0,0,0])
	
	# load the other agents to the problem solver
	def loadOtherAgents(self):
		# print 'There are ', len(self.platform.agents), 'agents'
		#load ghost agents
		for a in self.platform.agents:
			if (a.index != self.index):
				# if it is not itself then load a ghost agent
				g = Ghost()
				self.ps.loadObstacleFromUrdf(g.packageName, g.urdfName, a.robot.name) # it's the robot's name!!!
				# and then place it at the initial location of the agent
				# print self.robot.name, ' is now loading ', a.robot.name, ' as a ghost'
				config = a.current_config
				spec = self.getMoveSpecification(config)
				spec [2] = 0.3 
				self.obstacle.moveObstacle(a.robot.name + 'base_link_0', spec)
	
	# load agents from the node
	def loadOtherAgentsFromNode(self, node):
		print 'There are ', len(self.platform.agents), 'agents'
		#load ghost agents
		for a in self.platform.agents:
			if (a.index != self.index):
				# if it is not itself then load a ghost agent
				g = Ghost()
				self.ps.loadObstacleFromUrdf(g.packageName, g.urdfName, a.robot.name) # it's the robot's name!!!
				# and then place it at the initial location of the agent
				config = node.getAgentCurrentConfig(a.index)
				spec = self.getMoveSpecification(config)
				self.obstacle.moveObstacle(a.robot.name + 'base_link_0', spec)
				print self.robot.name, ' is now loading ', a.robot.name, ' as a ghost', 'it is at ', spec [0], spec [1]

	# note that the default solver does not consider the position of other agents
	def startDefaultSolver(self):
		self.repeat += 1
		name = self.robot.name
		self.problem.selectProblem(str(self.index)+' '+ str(self.repeat))
		self.robot = HyQ(name)
		self.ps = ProblemSolver(self.robot)
		self.ps.setInitialConfig(self.start_config)
		self.ps.addGoalConfig (self.end_config)
		self.ps.selectPathPlanner ("VisibilityPrmPlanner")
		self.ps.addPathOptimizer ("RandomShortcut")

	# initialise a solver from a node, the node contains information about other agents and itself
	# this method is used when proposing plans while interacting with platform for MAS path planning
	def startNodeSolver(self, node):
		self.repeat += 1
		name = self.robot.name
		self.problem.selectProblem(str(self.index)+' '+ str(self.repeat))
		self.robot = HyQ(name)
		self.ps = ProblemSolver(self.robot)
		cfg = node.getAgentCurrentConfig(self.index)
		print 'this iteration, the agent', name, 'starts from ', cfg[0], cfg[1]
		self.ps.setInitialConfig(cfg)
		self.ps.addGoalConfig (self.end_config)
		self.ps.selectPathPlanner ("VisibilityPrmPlanner")
		self.ps.addPathOptimizer ("RandomShortcut")

	# this is only used when the agent takes too long (30 seconds) while planning
	def terminate_solving(self):
		self.problem.interruptPathPlanning ()

	# the solve method for problem solver but with a time bound
	def solve(self):
		# try catch -------------------
		try: 
			t = Timer (30.0, self.terminate_solving)
			t.start()
			print 'solved: ', self.ps.solve()
			t.cancel()
		except Error as e:
			print e.msg
			print '***************\nfailed to plan within limited time\n**************'
			return -1

		# self.repeat += 1

	# store the path for two reasons:
	# 1. store as a default plan
	# 2. to continue the path
	def storePath(self, choice = 0, segments = 8):
		# always store the first one for now
		self.__plan_proposed = []
		for p in range(int(round(segments * self.ps.pathLength(choice)))):
			self.__plan_proposed.append(self.ps.configAtParam(choice, p* 1.0 / segments))
		
		# the last configuration is the goal configuration
		if self.ps.configAtParam(choice, self.ps.pathLength(choice)) == self.end_config:
			self.__plan_proposed.append(self.end_config)
		print 'stored; plan length: ', len(self.__plan_proposed)

	# this is hard colded for now for the airplane example, we should introduce an entry for the environment
	# class about it. 
	def setBounds(self):
		self.robot.setJointBounds ("base_joint_xy", [-35,10, -2.6, 4.3])

	#the rest are just some helping functions
	def getConfigOfProposedPlanAtTime(self, index):
		return self.__plan_proposed[index]

	def getConfigOfPermittedPlanAtTime(self, index):
		return self.permitted_plan[index]

	def getProposedPlanLength(self):
		return len(self.__plan_proposed)

	def setPermittedPlan(self, plan):
		self.permitted_plan = plan

	def getPermittedPlanLength(self):
		return len(self.permitted_plan)

	# export the permitted plan to a specific file in the format
	# 	agent X
	# 	config 1
	# 	config 2
	# 	etc
	def exportPermittedPlan(self, filename):
		f = open(filename, 'a+')
		f.write('agent ' + str(self.index) + '\n')
		for p in self.permitted_plan:
			f.write(str(p)[1:-1] + '\n')
		f.close()


	# for the sake of manipulation, we return a copy of it
	def obtainPermittedPlan(self):
		return copy.copy(self.permitted_plan)

		# we will get only a copy of it, not the original one 
		# to remind the difference, we use 'obtain' instead of 'get'
	def obtainProposedPlan(self):
		return copy.copy(self.__plan_proposed) #for some reason, sometimes the value would maybe changed???

	# to transfer the specification from 2D to 3D
	def getMoveSpecification(self, config):
		x = config[0]
		y = config[1]
		th = atan2(config[3], config[2]) 
		# print 'sin = ', self.init_config[3], ' cos = ', self.init_config[2], ' th = ', th
		return [x, y, 0, cos(th / 2) , 0, 0, sin(th / 2)]

	# the function to compute a plan, exceptions are not handled in this simple demo
	def computePlan(self, node):
		self.startNodeSolver(node)
		self.setBounds()
		self.setEnvironment()
		self.loadOtherAgentsFromNode(node)
		if self.solve() != -1:
			self.storePath()
		else:
			self.__plan_proposed = self.__plan_proposed[node.progress_time::]
			[node.getAgentCurrentConfig(self.index)]
			print 'take the previous one and continue the searching'
			return -1


		


