# Robert White
# Gepetto team, Robotics Department, LAAS-CNRS, France
# ILLC, University of Amsterdam, The Netherlands
# ai.robert.wangshuai@gmail.com


from Obstacle import Obstacle

# see the bigbox file in the repo
class Ghost (Obstacle):
	def __init__(self, ):
		self.name = 'ghost'
		self.packageName = 'hpp_tutorial'
		self.urdfName = 'bigbox'


