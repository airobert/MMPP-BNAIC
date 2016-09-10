# Robert White
# Gepetto team, Robotics Department, LAAS-CNRS, France
# ILLC, University of Amsterdam, The Netherlands
# ai.robert.wangshuai@gmail.com

# a class to deal with other objects
class Obstacle(object):
	name = ""
	packageName = ""
	urdfName = ""
	baseJointName = ""
	config  = []

	def __init__(self, name, packageName, urdfName, baseJointName):
		print 'create an obstacle/environment'
		self.name = name
		self.packageName = packageName
		self.urdfName = urdfName
		self.baseJointName = baseJointName

	def set_config(self, config):
		self.config = config

