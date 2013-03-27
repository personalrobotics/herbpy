import cbirrt, chomp, logging, openravepy
from planner import PlanningError 

def LookAt(robot, target, execute=True):
    # Find an IK solution to look at the point.
    ik_params = openravepy.IkParameterization(target, openravepy.IkParameterization.Type.Lookat3D)
    target_dof_values = robot.head.ik_database.manip.FindIKSolution(ik_params, 0)
    if target_dof_values == None:
        return None

    # Create a two waypoint trajectory for the head.
    current_dof_values = robot.GetDOFValues(robot.head.GetArmIndices())
    config_spec = robot.head.GetArmConfigurationSpecification()
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
    traj.Init(config_spec)
    traj.Insert(0, current_dof_values, config_spec)
    traj.Insert(1, target_dof_values, config_spec)

    # Optionally exeucute the trajectory.
    if execute:
        robot.head.arm_controller.SetPath(traj)
        # TODO: Implement a more efficient way of waiting for a single
        # controller to finish.
        while not robot.head.arm_controller.IsDone():
            pass

    return traj

def PlanGeneric(robot, command_name, execute=True, *args, **kw_args):
    traj = None

    # Sequentially try each planner until one succeeds.
    with robot.GetEnv():
        saver = robot.CreateRobotStateSaver()

        for planner in robot.planners:
            try:
                command = getattr(planner, command_name)
                print 'args:', args, kw_args
                print 'cmd:', command
                traj = command(*args, **kw_args)
                break
            except NotImplementedError:
                pass
            except PlanningError, e:
                logging.warning('Planning with {0:s} failed: {1:s}'.format(planner.GetName(), e))

        del saver

    if traj is None:
        logging.error('Planning failed with all planners.')
        return None

    # TODO: Retime the trajectory.
    # TODO: Optionally execute the trajectory.
    return traj

def PlanToConfiguration(robot, goal, **kw_args):
    return PlanGeneric(robot, 'PlanToConfiguration', robot, goal, **kw_args)

def PlanToEndEffectorPose(robot, goal_pose, **kw_args):
    return PlanGeneric(robot, 'PlanToEndEffectorPose', robot, goal_pose, **kw_args)

def BlendTrajectory(robot, traj, **kw_args):
    with robot.GetEnv():
        saver = robot.CreateRobotStateSaver()
        return robot.trajectory_module.blendtrajectory(traj=traj, execute=False, **kw_args)
