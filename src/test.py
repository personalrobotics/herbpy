import herbpy, openravepy, numpy

right_home_config = numpy.array([ 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00 ])
right_relaxed_config = numpy.array([ 5.73, -1.82, -0.35, 1.87, -4.06, -0.66, 0.98 ])

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
openravepy.misc.InitOpenRAVELogging();

env, robot = herbpy.initialize(left_arm_sim=False, right_arm_sim=False,
                               left_hand_sim=False, right_hand_sim=False,
                               head_sim=False, segway_sim=False,
                               moped_sim=True, attach_viewer=True)

def look_at_hand(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    robot.LookAt(target, execute=True)

with env:
    robot.SetActiveManipulator(robot.right_arm)
    robot.SetActiveDOFs(robot.right_arm.GetArmIndices())
'''
    robot.SetActiveDOFValues(right_start_config)
    robot.SetTransform([[ 1, 0, 0, -1.25 ],
                        [ 0, 1, 0,  0.49 ],
                        [ 0, 0, 1,  0.00 ],
                        [ 0, 0, 0,     1 ]])
'''

# TODO: Named configurations.
# TODO: Finger control.
# TODO: Executing trajectories through multicontroller is broken.
# TODO: Executing blending trajectories is broken.
# FIXME: CHOMP always returns success.
traj = robot.cbirrt_planner.PlanToConfiguration(right_home_config)
blended_traj = robot.BlendTrajectory(traj)
annotated_traj = robot.AddTrajectoryFlags(blended_traj, stop_on_stall=True)
#robot.ExecuteTrajectory(annotated_traj)

# test cbirrt
'''
robot.SetActiveDOFValues(right_home_config)
Tstart = robot.GetActiveManipulator().GetEndEffectorTransform()
Tend = numpy.array(Tstart)
Tend[2, 3] -= 0.2
traj = robot.cbirrt_planner.PlanToEndEffectorPose(Tend)
'''
