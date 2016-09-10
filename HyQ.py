# Robert White
# Gepetto team, Robotics Department, LAAS-CNRS, France
# ILLC, University of Amsterdam, The Netherlands
# ai.robert.wangshuai@gmail.com


from hpp.corbaserver.robot import Robot

class HyQ (Robot):
	robotType = "hyq"
	packageName = "hyq_description"
	meshPackageName = "hyq_description"
	rootJointType = "planar"
	urdfName = "hyq"
	urdfSuffix = ""
	srdfSuffix = ""
	def __init__(self, agentName):
		print 'initialising a HyQ agent'
		Robot.__init__(self, agentName, self.rootJointType)
		# self.set_init(0, 0)
		
	# the initial configuration is invalid, this is a valid configuration
	def setInit(self, x, y):
		print 'the agent is now set to its initial configuration at (', x, ', ', y, ')'
		self.init_config = self.getCurrentConfig()
		self.init_config[0] = x
		self.init_config[1] = y
		self.init_config[6] = -0.5
		self.init_config[9] = 0.5
		self.init_config[12] = -0.5
		self.init_config[15] = 0.5
		self.setCurrentConfig(self.init_config)
		# self.platform.r(self.init_config)
		# error message

	# def set_config(x, y): 


