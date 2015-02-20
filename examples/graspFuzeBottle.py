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
robot.PlanToNamedConfiguration('relaxed_home') # move the arms to the 'relaxed_home' position

# drive to the table
robot_in_table = numpy.array([[0., 1., 0.,  0.], 
                              [0., 0., 1.,  0.],
                              [1., 0., 0., -1.025],
                              [0., 0., 0.,  1.]])
base_pose = numpy.dot(table.GetTransform(), robot_in_table)
base_pose[2,3] = 0
robot.base.PlanToBasePose(base_pose)
# robot.SetTransform(base_pose) # way faster for testing

# get a pose for grabbing the fuze bottle
pose_near_fuze = fuze.GetTransform()
pose_near_fuze[:3,:3] = [ [ math.cos(math.pi/6), -math.sin(math.pi/6), 0 ], # z rotation matrix
                           [ math.sin(math.pi/6), math.cos(math.pi/6), 0 ],
                           [ 0, 0, 1 ] ]
pose_near_fuze[2,3] += .1 # move up a bit so we're grasping the middle (not the bottom) of the bottle
pose_near_fuze[:3,:3] = numpy.dot(pose_near_fuze[:3,:3], [ [ 1, 0, 0 ], # x rotation matrix
                                                        [ 0, math.cos(math.pi/2), -math.sin(math.pi/2) ],
                                                        [ 0, math.sin(math.pi/2), math.cos(math.pi/2) ], ])
pose_near_fuze[:3,3] += numpy.dot(pose_near_fuze[:3,:3], [0, 0, -.21]) # move away from bottle, pose is centered at wrist not palm

# visualize the pose
Axes = openravepy.misc.DrawAxes(env, pose_near_fuze)
Axes.SetShow(True)
raw_input('press enter to stop visualizing the pose')
Axes.SetShow(False)

# plan the end effector to the pose
config = robot.right_arm.FindIKSolution(pose_near_fuze, openravepy.IkFilterOptions.CheckEnvCollisions)
if config == None:
    raw_input('couldn\'t find config, next line is going to fail...')
robot.right_arm.PlanToConfiguration(config)

# grab the fuze bottle
robot.right_arm.hand.CloseHand()
robot.right_arm.SetActive()
robot.Grab(fuze)
robot.right_arm.PlanToNamedConfiguration('home')

# we do this so the viewer doesn't close when the example is done
import IPython; IPython.embed()
