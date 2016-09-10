# Robert White
# Gepetto team, Robotics Department, LAAS-CNRS, France
# ILLC, University of Amsterdam, The Netherlands
# ai.robert.wangshuai@gmail.com

# just some additional environment models for testing
from Obstacle import Obstacle

class Environment(Obstacle):

	def __init__(self, name, packageName, urdfName, baseJointName):
		Obstacle.__init__ (self, name, packageName, urdfName, baseJointName)

# a kitchen as in the tutorial
class Kitchen(Environment):

	def __init__(self, name):
		# vf.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
		self.name = name
		self.packageName = "iai_maps"
		self.urdfName = "kitchen_area"
		Environment.__init__(self, self.name, self.packageName, self.urdfName, "kitchen_base_joint")

#a basic environment
class BasicHouse(Environment):
	def __init__(self, name):
		# vf.loadObstacleModel ("iai_maps", "kitchen_area", "kitchen")
		self.name = name
		self.packageName = "hpp-rbprm-corba"
		self.urdfName = "basic"
		Environment.__init__(self, self.name, self.packageName, self.urdfName, "basic")

		
class Airplane(Environment):
	def __init__(self, name):
		self.name = name
		self.packageName = "hpp-rbprm-corba"
		self.urdfName = "simple_boeing"
		Environment.__init__(self, self.name, self.packageName, self.urdfName, self.name)