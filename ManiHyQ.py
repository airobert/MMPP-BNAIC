
from hpp.corbaserver.manipulation.robot import  Robot
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
		Robot.__init__(self, 'meta', agentName, "planar")


