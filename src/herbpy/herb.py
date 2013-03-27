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

def RetimeTrajectory(robot, traj, maxsmoothiter=None, resolution=None, blend_radius=0.2,
                     blend_attempts=4, blend_step_size=0.05, linearity_threshold=0.1, 
                     ignore_collisions=None ):
    args = [ 'ExecuteBlendedTrajectory' ]
    args += [ 'execute', '0' ]
    args += [ 'traj', traj.serialize(0) ]
    
    if maxsmoothiter is not None:
        args += [ 'maxsmoothiter', str(maxsmoothiter) ]
    if resolution is not None:
        args += [ 'resolution', ' '.join([ '{0:.04f}'.format(r) for r in resolution ]) ]
    if blend_radius is not None:
        args += [ 'blend_radius', str(blend_radius) ]
    if blend_attempts is not None:
        args += [ 'blend_attempts', str(blend_attempts) ]
    if blend_step_size is not None:
        args += [ 'blend_step_size', str(blend_step_size) ]
    if linearity_threshold is not None:
        args += [ 'linearity_threshold', linearity_threshold ]
    if ignore_collisions is not None:
        args += [ 'ignore_collisions', str(int(ignore_collisions)) ]

    args_str = ' '.join(args)
    timed_traj_xml = self.trajectory_problem.SendCommand(args_str)

    if not timed_traj_xml:
        logger.warning('Trajectory retiming failed.')
        return None

    timed_traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
    timed_traj.deserialize(timed_traj_xml)
    return timed_traj 
