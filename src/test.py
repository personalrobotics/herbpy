import herbpy, openravepy, numpy

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
openravepy.misc.InitOpenRAVELogging();

'''
env, robot = herbpy.initialize(left_arm_sim=False, right_arm_sim=False,
                               left_hand_sim=False, right_hand_sim=False,
                               head_sim=False, segway_sim=True,
                               moped_sim=True, attach_viewer=True)
'''
env, robot = herbpy.initialize_sim(attach_viewer=True)

def look_at_hand(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    herbpy.look_at(robot, target, execute=True)

robot.SetActiveManipulator(robot.right_arm)
robot.SetActiveDOFs(robot.right_arm.GetArmIndices())

right_home_config = [ 3.68, -1.90,  0.00,  2.20,  0.00,  0.00, 0.00 ]

# TODO: Trajectory retiming.
# FIXME: Executing CHOMP trajectories through multicontroller fails.

# test cbirrt
'''
robot.SetActiveDOFValues(right_home_config)
Tstart = robot.GetActiveManipulator().GetEndEffectorTransform()
Tend = numpy.array(Tstart)
Tend[2, 3] -= 0.2
traj = robot.cbirrt_planner.PlanToEndEffectorPose(Tend)
'''

#traj = robot.chomp_planner.PlanToConfiguration(right_home_config)

