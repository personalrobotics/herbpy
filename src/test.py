import herbpy, openravepy, numpy

right_start_config = [ 4.6, -1.57, 0.0, 2.8, -2.5, -0.4, 0.0 ]
right_home_config = [ 3.68, -1.90,  0.00,  2.2,  0.00,  0.00, 0.00 ]

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

with env:
    robot.SetActiveManipulator(robot.right_arm)
    robot.SetActiveDOFs(robot.right_arm.GetArmIndices())
    robot.SetActiveDOFValues(right_start_config)
    robot.SetTransform([[ 1, 0, 0, -1.25 ],
                        [ 0, 1, 0,  0.49 ],
                        [ 0, 0, 1,  0.00 ],
                        [ 0, 0, 0,     1 ]])

# TODO: Named configurations.
# TODO: Finger control.
# FIXME: CHOMP always returns success.
traj = robot.PlanToConfiguration(right_home_config)#, n_iter=100, lambda_=100.0)
blended_traj = robot.BlendTrajectory(traj)
#robot.ExecuteTrajectory(blended_traj)

# test cbirrt
'''
robot.SetActiveDOFValues(right_home_config)
Tstart = robot.GetActiveManipulator().GetEndEffectorTransform()
Tend = numpy.array(Tstart)
Tend[2, 3] -= 0.2
traj = robot.cbirrt_planner.PlanToEndEffectorPose(Tend)
'''
