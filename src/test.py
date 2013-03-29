import herbpy, openravepy, numpy

def look_at_hand(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    robot.LookAt(target, execute=True)

simulation = False 
right_home_config = numpy.array([ 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00 ])
right_relaxed_config = numpy.array([ 5.73, -1.82, -0.35, 1.87, -4.06, -0.66, 0.98 ])

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
openravepy.misc.InitOpenRAVELogging();

if simulation:
    env, robot = herbpy.initialize_sim(attach_viewer=True)
else:
    env, robot = herbpy.initialize(left_arm_sim=False, right_arm_sim=False,
                                   left_hand_sim=False, right_hand_sim=False,
                                   head_sim=False, segway_sim=False,
                                   moped_sim=True, attach_viewer=True)

with env:
    robot.SetActiveManipulator(robot.right_arm)
    robot.SetActiveDOFs(robot.right_arm.GetArmIndices())

if simulation:
    robot.SetActiveDOFValues(right_relaxed_config)
    robot.SetTransform([[ 1, 0, 0, -1.25 ],
                        [ 0, 1, 0,  0.49 ],
                        [ 0, 0, 1,  0.00 ],
                        [ 0, 0, 0,     1 ]])

robot.chomp_planner.ComputeDistanceField()

#traj = robot.cbirrt_planner.PlanToConfiguration(right_home_config)

# TODO: Named configurations.
# TODO: Wait for the hand to finish closing.
# TODO: Add F/T tare functionality.
# TODO: Don't compute the signed distance field by default.
# FIXME: WaitForController doesn't work.
# FIXME: CHOMP always returns success.

if False:
    goal_pose = robot.right_arm.GetEndEffectorTransform()
    goal_pose[0:3, 3] += numpy.array([ 0, 0, 0.4 ])

    traj = robot.cbirrt_planner.PlanToEndEffectorPose(goal_pose)
    blended_traj = robot.BlendTrajectory(traj)
    annotated_traj = robot.AddTrajectoryFlags(blended_traj, stop_on_stall=True,
        stop_on_ft=True, force_direction=[0,-1,0], force_magnitude=5, torque=[100,100,100]
    )
    robot.ExecuteTrajectory(annotated_traj)
