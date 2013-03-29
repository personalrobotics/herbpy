import cbirrt, chomp, logging, openravepy
import numpy
import planner
from methodlist_decorator import CreateMethodListDecorator

HerbMethod = CreateMethodListDecorator()

@HerbMethod
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

@HerbMethod
def PlanGeneric(robot, command_name, args, execute=True, **kw_args):
    traj = None

    # Sequentially try each planner until one succeeds.
    with robot.GetEnv():
        with robot.CreateRobotStateSaver():
            for delegate_planner in robot.planners:
                try:
                    traj = getattr(delegate_planner, command_name)(*args, **kw_args)
                    break
                except planner.UnsupportedPlanningError, e:
                    logging.debug('Unable to plan with {0:s}: {1:s}'.format(delegate_planner.GetName(), e))
                except planner.PlanningError, e:
                    logging.warning('Planning with {0:s} failed: {1:s}'.format(delegate_planner.GetName(), e))

    if traj is None:
        logging.error('Planning failed with all planners.')
        return None

    # Optionally execute the trajectory.
    if execute:
        return robot.ExecuteTrajectory(traj, **kw_args)
    else:
        return traj

@HerbMethod
def MoveUntilTouch(robot, direction, distance, max_force=5, execute=True):
    # Compute the expected force direction in the hand frame.
    direction = numpy.array(direction)
    hand_pose = robot.GetActiveManipulator().GetEndEffectorTransform()
    force_direction = numpy.dot(hand_pose[0:3, 0:3], -direction)

    # Plan a straight trajectory.
    traj = robot.PlanToEndEffectorOffset(direction, distance, execute=False)
    traj = robot.AddTrajectoryFlags(traj, stop_on_ft=True, force_direction=force_direction,
                                          force_magnitude=max_force, torque=[100,100,100])

    if execute:
        return robot.ExecuteTrajectory(traj)
    else:
        return traj

@HerbMethod
def PlanToNamedConfiguration(robot, name):
    pass

@HerbMethod
def BlendTrajectory(robot, traj, **kw_args):
    with robot.GetEnv():
        saver = robot.CreateRobotStateSaver()
        return robot.trajectory_module.blendtrajectory(traj=traj, execute=False, **kw_args)

@HerbMethod
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

@HerbMethod
def ExecuteTrajectory(robot, traj, timeout=None, blend=True, retime=True):
    # Annotate the trajectory with HERB-specific options.
    if blend:
        traj = robot.BlendTrajectory(traj)

    if retime:
        logging.warning('Trajectory retiming is not supported.')

    # Only add flags if none are present. This is the only way of checking if
    # the trajectory already has the flags added.
    try:
        config_spec = traj.GetConfigurationSpecification()
        group = config_spec.GetGroupFromName('or_owd_controller')
    except openravepy.openrave_exception:
        traj = robot.AddTrajectoryFlags(traj, stop_on_stall=True)

    # FIXME: Waiting for the controller fails.
    robot.GetController().SetPath(traj)
    if timeout == None:
        robot.WaitForController(0)
    elif timeout > 0:
        robot.WaitForController(timeout)

    return traj
