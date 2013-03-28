import cbirrt, chomp, logging, openravepy
import numpy
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

    # Retiming the trajectory is necessary for it play on the IdealController.
    openravepy.planningutils.RetimeTrajectory(traj)

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

def PlanToNamedConfiguration(robot, name):
    pass

def BlendTrajectory(robot, traj, **kw_args):
    with robot.GetEnv():
        saver = robot.CreateRobotStateSaver()
        return robot.trajectory_module.blendtrajectory(traj=traj, execute=False, **kw_args)

def AddTrajectoryFlags(robot, traj, stop_on_stall=True, stop_on_ft=False,
                       force_direction=None, force_magnitude=None, torque=None):
    flags  = [ 'or_owd_controller' ]
    flags += [ 'stop_on_stall', str(int(stop_on_stall)) ]
    flags += [ 'stop_on_ft', str(int(stop_on_ft)) ]

    if stop_on_ft:
        if force_direction is None:
            logging.error('Force direction must be specified if stop_on_ft is true.')
            return None
        elif force_magnitude is None:
            logging.error('Force magnitude must be specified if stop_on_ft is true.')
            return None 
        elif torque is None:
            logging.error('Torque must be specified if stop_on_ft is true.')
            return None 
        elif len(force_direction) != 3:
            logging.error('Force direction must be a three-dimensional vector.')
            return None
        elif len(torque) != 3:
            logging.error('Torque must be a three-dimensional vector.')
            return None

        flags += [ 'force_direction' ] + [ str(x) for x in force_direction ]
        flags += [ 'force_magnitude', str(force_magnitude) ]
        flags += [ 'torque' ] + [ str(x) for x in torque ]

    # Add a bogus group to the trajectory to hold these parameters.
    flags_str = ' '.join(flags)
    config_spec = traj.GetConfigurationSpecification();
    group_offset = config_spec.AddGroup(flags_str, 1, 'next')

    annotated_traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
    annotated_traj.Init(config_spec)
    for i in xrange(traj.GetNumWaypoints()):
        waypoint = numpy.zeros(config_spec.GetDOF())
        waypoint[0:-1] = traj.GetWaypoint(i)
        annotated_traj.Insert(i, waypoint)

    return annotated_traj

def ExecuteTrajectory(robot, traj, timeout=None):
    robot.GetController().SetPath(traj)
    if timeout == None:
        robot.WaitForController(0)
    elif timeout > 0:
        robot.WaitForController(timeout)

def TareForceTorqueSensor(robot):
    #
    pass
