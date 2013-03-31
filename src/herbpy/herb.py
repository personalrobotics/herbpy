import logging, openravepy
import numpy
import planner
import time
import util

HerbMethod = util.CreateMethodListDecorator()

@HerbMethod
def LookAt(robot, target, execute=True):
    """
    Look at a point in the world frame. This creates and, optionally executes,
    a two-waypoint trajectory that starts at the current configuration and
    moves to a goal configuration found through the neck's inverse kinematics.
    @param target point in the world frame.
    @param execute immediately execute the trajectory
    @return trajectory head trajectory
    """
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
        return robot.ExecuteTrajectory(traj, retime=True)
    else:
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
                    # TODO: Log the scene and planner parameters to a file.

    if traj is None:
        raise planner.PlanningError('Planning failed with all planners.')

    # Optionally execute the trajectory.
    if execute:
        return robot.ExecuteTrajectory(traj, **kw_args)
    else:
        return traj

@HerbMethod
def PlanToNamedConfiguration(robot, name, **kw_args):
    config_inds = numpy.array(robot.configs[name]['dofs'])
    config_vals = numpy.array(robot.configs[name]['vals'])

    if len(config_inds) == 0:
        raise Exception('Failed to find named config: %s'%name)

    # TODO: Hacky. Can we do this better?
    # Need to parse out left and right manipulator indices
    # Also do we want to parse out head?
    all_left = numpy.array(robot.left_arm.GetArmIndices())
    all_left.sort()    
    all_right = numpy.array(robot.right_arm.GetArmIndices())
    all_right.sort()

    left_vals = []
    left_inds = []
    right_vals = []
    right_inds = []

    for dof, val in zip(config_inds, config_vals):
        if dof in all_left:
            left_inds.append(dof)
            left_vals.append(val)
        if dof in all_right:
            right_inds.append(dof)
            right_vals.append(val)

    traj_left = None
    traj_right = None
    if len(left_inds) > 0:
        robot.SetActiveDOFs(left_inds)
        traj_left = robot.left_arm.PlanToConfiguration(left_vals, **kw_args)
    if len(right_inds) > 0:
        robot.SetActiveDOFs(right_inds)
        traj_right = robot.right_arm.PlanToConfiguration(right_vals, **kw_args)

    return [ traj_left, traj_right ]

@HerbMethod
def AddNamedConfiguration(robot, name, dofs, vals):
    if len(dofs) != len(vals):
        raise Exception('AddNamedConfiguration Failed. Lengths of dofs and vals must be equal:\n\tconfig_inds=%s\n\tconfig_vals=[%s]'%(str(config_indxs), SerializeArray(config_vals)))

    robot.configs[name] = {}
    robot.configs[name]['dofs'] = numpy.array( dofs )
    robot.configs[name]['vals'] = numpy.array( vals )

@HerbMethod
def BlendTrajectory(robot, traj, maxsmoothiter=None, resolution=None,
                    blend_radius=0.2, blend_attempts=4, blend_step_size=0.05,
                    linearity_threshold=0.1, ignore_collisions=None, **kw_args):
    """
    Blend a trajectory. This appends a blend_radius group to an existing
    trajectory.
    @param traj input trajectory
    @return blended_trajectory trajectory with additional blend_radius group
    """
    with robot.GetEnv():
        saver = robot.CreateRobotStateSaver()
        return robot.trajectory_module.blendtrajectory(traj=traj, execute=False,
            maxsmoothiter=maxsmoothiter, resolution=resolution,
            blend_radius=blend_radius, blend_attempts=blend_attempts,
            blend_step_size=blend_step_size, linearity_threshold=linearity_threshold,
            ignore_collisions=ignore_collisions
        )

@HerbMethod
def AddTrajectoryFlags(robot, traj, stop_on_stall=True, stop_on_ft=False,
                       force_direction=None, force_magnitude=None, torque=None):
    """
    Add OWD trajectory execution options to a trajectory. These options are
    encoded in the or_owd_controller group. The force_direction, force_magnitude,
    and torque parameters must be specified if stop_on_ft is True.
    @param traj input trajectory
    @param stop_on_stall stop the trajectory if the stall torques are exceeded
    @param stop_on_ft stop the trajectory on force/torque sensor input
    @param force_direction unit vector of the expected force in the hand frame
    @param force_magnitude maximum force magnitude in meters
    @param torque maximum torque in the hand frame in Newton-meters
    @return annotated_traj trajectory annotated with OWD execution options
    """
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
def ExecuteTrajectory(robot, traj, timeout=None, blend=True, retime=False, **kw_args):
    """
    Execute a trajectory. By default, this retimes, blends, and adds the
    stop_on_stall flag to all trajectories. Additionally, this function blocks
    until trajectory execution finishes. This can be changed by changing the
    timeout parameter to a maximum number of seconds. Pass a timeout of zero to
    return instantly.
    @param traj trajectory to execute
    @param timeout blocking execution timeout
    @param blend compute blend radii before execution
    @param retime retime the trajectory before execution
    @return executed_traj  
    """
    # Retiming the trajectory may be necessary to execute it on an
    # IdealController in simulation. This timing is ignored by OWD.
    if retime:
        openravepy.planningutils.RetimeTrajectory(traj)

    # Annotate the trajectory with HERB-specific options.
    if blend:
        traj = robot.BlendTrajectory(traj)

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

@HerbMethod
def WaitForObject(robot, obj_name, timeout=None, update_period=0.1):
    start = time.time()
    found_body = None

    # TODO: This should be wrapped elsewhere.
    robot.moped_sensorsystem.SendCommand('Enable')

    try:
        while True:
            logging.info("Waiting for object %s to appear.", obj_name)

            # Check for an object with the appropriate name in the environment.
            bodies = robot.GetEnv().GetBodies()
            for body in bodies:
                if body.GetName().startswith('moped_' + obj_name):
                    return body

            # Check for a timeout.
            if timeout is not None and time.time() - start >= timeout:
                logging.info("Timed out without finding object.")
                return None

            time.sleep(update_period)
    finally:
        robot.moped_sensorsystem.SendCommand('Disable')

@HerbMethod
def DriveStraightUntilForce(robot, direction=[0.0,0.0,0.0], max_distance=1.0, right_arm=True, left_arm=True, force_threshold=3.0):
    controller_name = robot.segway_controller.GetXMLId().split()[0]
    if controller_name == 'IdealController':
        logger.error("drive_segway_until_force not working if herbcontroller is not used.")
        raise Exception
    else:
        

        # Tare the appropariate ft sensors
        if right_arm:
            robot.right_arm.TareForceTorqueSensor()
        if left_arm:
            robot.left_arm.TareForceTorqueSensor()

        # rotate to face the right direction
        robot_pose = robot.GetTransform()
        robot_angle = numpy.arctan2(robot_pose[1,0], robot_pose[0,0])
        des_angle = numpy.arctan2(direction[1], direction[0])
        robot.RotateSegway(des_angle - robot_angle)
        
        # now drive until we feel a force
        robot.segway_controller.SendCommand("Drive " + str(max_distance))
        felt_force = False
        start = time.time()
        while not felt_force:

            if right_arm:
                force, torque = robot.right_arm.GetForceTorque()
                if numpy.sqrt(numpy.dot(force, force)) > force_threshold:
                    # stop the segway
                    robot.segway_controller.Reset(0)
                    felt_force = True
            if left_arm:
                force, torque = robot.left_arm.GetForceTorque()
                if numpy.sqrt(numpy.dot(force, force)) > force_threshold:
                    # stop the segway
                    robot.segway_controller.Reset(0)
                    felt_force = True

        return felt_force

@ HerbMethod
def DriveAlongVector(robot, direction, goal_pos):
    direction = direction[:2]/numpy.linalg.norm(direction[:2])
    herb_pose = robot.GetTransform()
    distance = numpy.dot(goal_pos[:2]-herb_pose[:2,3], direction)
    cur_angle = numpy.arctan2(herb_pose[1,0],herb_pose[0,0])
    des_angle = numpy.arctan2(direction[1],direction[0])
    robot.RotateSegway(des_angle-cur_angle)
    robot.DriveSegway(distance)


@HerbMethod
def DriveSegway(robot, meters, timeout=None):
    
    controller_name = robot.segway_controller.GetXMLId().split()[0]
    if controller_name == 'IdealController':
        # in simulation
        current_pose = robot.GetTransform().copy()
        current_pose[0:3,3] = current_pose[0:3,3] + meters*current_pose[0:3,0]
        robot.SetTransform(current_pose)

    else:
        robot.segway_controller.SendCommand("Drive " + str(meters))
        if timeout == None:
            robot.WaitForController(0)
        elif timeout > 0:
            robot.WaitForController(timeout)

        

@HerbMethod
def DriveSegwayToNamedPosition(robot, named_position):
    
    controller_name = robot.segway_controller.GetXMLId().split()[0]
    if controller_name == 'IdealController':
        logger.warm('Drive to named positions not implemented in simulation.')
    else:
        robot.segway_controller.SendCommand("Goto " + named_position)

@HerbMethod
def RotateSegway(robot, angle_rad, timeout=None):
    
    controller_name = robot.segway_controller.GetXMLId().split()[0]
    if controller_name == 'IdealController':
        # in simulation
        current_pose_in_world = robot.GetTransform().copy()
        
        #rotate by the angle around z
        desired_pose_in_herb = numpy.array([[numpy.cos(angle_rad), -numpy.sin(angle_rad), 0, 0],
                                            [numpy.sin(angle_rad), numpy.cos(angle_rad), 0, 0],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]])
        desired_pose_in_world = numpy.dot(current_pose_in_world, desired_pose_in_herb)
        robot.SetTransform(desired_pose_in_world)
    else:
        robot.segway_controller.SendCommand("Rotate " + str(angle_rad))
        if timeout == None:
            robot.WaitForController(0)
        elif timeout > 0:
            robot.WaitForController(timeout)
