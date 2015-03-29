import herbpy
import os
import numpy
import math
import openravepy

# get the location of some objects (you'll need the pr_ordata package)
from catkin.find_in_workspaces import find_in_workspaces
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project='pr_ordata',
    path='data/objects',
    first_match_only=True)[0]


print objects_path # for me this is '/home/eashapir/ros-hydro/src/pr-ordata/data/objects'

# ===========================
#   ENVIRONMENT SETUP
# ===========================

env, robot = herbpy.initialize(sim=True, attach_viewer='rviz')

# add a table to the environment
table_file = os.path.join(objects_path, 'table.kinbody.xml')
table = env.ReadKinBodyXMLFile(table_file)
env.AddKinBody(table)
table_pose = numpy.array([[1., 0.,  0., 2],
                    [0., 0., -1., 2],
                    [0., 1.,  0., 0.0], 
                    [0., 0.,  0., 1.]])
table.SetTransform(table_pose)

# add a fuze bottle on top of the table
fuze_path = os.path.join(objects_path, 'fuze_bottle.kinbody.xml')
fuze = env.ReadKinBodyXMLFile(fuze_path)
table_aabb = table.ComputeAABB()
x = table_aabb.pos()[0] + table_aabb.extents()[0]*0 # middle of table in x
y = table_aabb.pos()[1] + table_aabb.extents()[1]*.6 # closer to one side of table in y
z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 # slightly above table in z (so its not in collision
fuze_pose = fuze.GetTransform()
fuze_pose[:3,3] = numpy.transpose([x, y, z])
fuze.SetTransform(fuze_pose)
env.AddKinBody(fuze)

# ===========================
#   PLANNING
# ===========================

raw_input('press enter to begin planning')

# move to a good start position
robot.head.MoveTo([0, -math.pi/16]) # look down slightly
#robot.PlanToNamedConfiguration('relaxed_home') # move the arms to the 'relaxed_home' position
indices, values = robot.configurations.get_configuration('relaxed_home') # Faster for testing
robot.SetDOFValues(values=values, dofindices=indices)

# drive to the table
robot_in_table = numpy.array([[0., 1., 0.,  0.], 
                              [0., 0., 1.,  0.],
                              [1., 0., 0., -1.025],
                              [0., 0., 0.,  1.]])
base_pose = numpy.dot(table.GetTransform(), robot_in_table)
base_pose[2,3] = 0
#robot.base.PlanToBasePose(base_pose)
robot.SetTransform(base_pose) # way faster for testing

# Grasp the bottle
robot.right_arm.Grasp(fuze)
robot.right_arm.PlanToNamedConfiguration('home')

# we do this so the viewer doesn't close when the example is done
import IPython; IPython.embed()
