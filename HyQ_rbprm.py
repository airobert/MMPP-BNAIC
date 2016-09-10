# gepetto-viewer-server
# not hpp-manipulation-server
# hppcorbaserver

from Agent import *
from Platform import *
from HyQ import HyQ
from Environment import Airplane

# agent 1 
r1 = HyQ('myself')

r1.setInit(3,2)

q_init = r1.getCurrentConfig()
q_goal = q_init[::]
q_goal[0] = -7.5
q_goal[1] = -2
q_goal[2] = 0 
q_goal[3] = 1


a1 = Agent(r1, q_init, q_goal)

# # agent 2 
r2 = HyQ('sister')
r1.setInit(4,0)
q_init = r2.getCurrentConfig()
q_goal = q_init[::]
q_goal[0] = -18
q_goal[1] = -1.7
q_goal[2] = 0 
q_goal[3] = 1
a2 = Agent(r2, q_init, q_goal)

# # agent 3

r3 = HyQ('brother')
r3.setInit(0,0)
q_init = r3.getCurrentConfig()
q_goal = q_init[::]
q_goal[0] = -7.5
q_goal[1] = 4
q_goal[2] = 0 
q_goal[3] = -1
a3 = Agent(r3, q_init, q_goal)

# # agent 4

r4 = HyQ('uncle')
r4.setInit(-9,2)
q_init = r4.getCurrentConfig()
q_goal = q_init[::]
q_goal[0] = -18
q_goal[1] = 4
q_goal[2] = 0
q_goal[3] = -1
a4 = Agent(r4, q_init, q_goal)


# # agent 5

# r5 = HyQ('aunty')
# r5.setInit(9, 2)
# q_init = r5.getCurrentConfig()
# q_goal = q_init[::]
# q_goal[0] = -18
# q_goal[1] = -1.7
# q_goal[2] = 0 
# q_goal[3] = -1
# a5 = Agent(r5, q_init, q_goal)


# platform
pl = Platform([a1, a2, a3, a4])


air = Airplane("air")
pl.setEnvironment(air)

pl.start()
pl.r.client.gui.addLight('0_scene_hpp_/x', pl.r.windowId,0.1, [1,1,1,1])
pl.r.client.gui.applyConfiguration('0_scene_hpp_/x', [-3,0,7,1,0,0,0])
pl.r.client.gui.setColor('air', [1,1,1,0.5])
pl.r.client.gui.refresh()


print 'start the searching with ', pl.tree.getAgentsRemained(), ' remained'

(result, plans) = pl.construct_tree(100)


a1.setPermittedPlan(plans[0])
a2.setPermittedPlan(plans[1])
a3.setPermittedPlan(plans[2])
a4.setPermittedPlan(plans[3])
# a3.setPermittedPlan(plans[4])


# pl.playAllPermittedPath()
 
filename = 'allPath.path'
a1.exportPermittedPlan(filename)
a2.exportPermittedPlan(filename)
a3.exportPermittedPlan(filename)
a4.exportPermittedPlan(filename)
# a5.exportPermittedPlan(filename)


# a1.startDefaultSolver()
# a1.setBounds()
# a1.setEnvironment()
# a1.loadOtherAgents()
# a1.solve()
# a1.storePath()

# a2.startDefaultSolver()
# a2.setBounds()
# a2.setEnvironment()
# a2.loadOtherAgents()
# a2.solve()
# a2.storePath()

# a3.startDefaultSolver()
# a3.setBounds()
# a3.setEnvironment()
# a3.loadOtherAgents()
# a3.solve()
# a3.storePath()


# pl.playAllPath()



