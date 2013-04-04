import herbpy
import exceptions
import openravepy
import numpy
import planner
import time
import util
import rospy
from pr_msgs.srv import AppletCommand

HerbMethod = util.CreateMethodListDecorator()

@HerbMethod
def Say(robot, message):
    """
    Say a message using HERB's text-to-speech engine.
    @param message
    """
    herbpy.logger.info('Saying "%s".', message)

    rospy.wait_for_service('/talkerapplet')
    talk = rospy.ServiceProxy('/talkerapplet', AppletCommand)    

    try:
        talk('say', message, 0, 0)
    except rospy.ServiceException, e:
        herbpy.logger.info('Error talking.')


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

    # Update the controllers to get new joint values.
    with robot.GetEnv():
        robot.GetController().SimulationStep(0)
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
def LookAtKinBody(robot, body):
    target = body.GetTransform()[0:3, 3]
    traj = robot.LookAt(target, execute=True)

@HerbMethod
def FindHeadDofs(robot, target):
    # Find an IK solution to look at the point.
    ik_params = openravepy.IkParameterization(target, openravepy.IkParameterization.Type.Lookat3D)
    target_dof_values = robot.head.ik_database.manip.FindIKSolution(ik_params, 0)
    if target_dof_values == None:
        return None
    return target_dof_values


@HerbMethod
def PlanGeneric(robot, command_name, args, execute=True, **kw_args):
    traj = None

    with robot.GetEnv():
        # Update the controllers to get new joint values.
        robot.GetController().SimulationStep(0)

        # Sequentially try each planner until one succeeds.
        with robot.CreateRobotStateSaver():
            for delegate_planner in robot.planners:
                try:
                    herbpy.logger.info('Trying planner: %s', delegate_planner.GetName())
                    traj = getattr(delegate_planner, command_name)(*args, **kw_args)
                    break
                except planner.UnsupportedPlanningError, e:
                    herbpy.logger.debug('Unable to plan with {0:s}: {1:s}'.format(delegate_planner.GetName(), e))
                except planner.PlanningError, e:
                    herbpy.logger.warning('Planning with {0:s} failed: {1:s}'.format(delegate_planner.GetName(), e))
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
            herbpy.logger.error('Force direction must be specified if stop_on_ft is true.')
            return None
        elif force_magnitude is None:
            herbpy.logger.error('Force magnitude must be specified if stop_on_ft is true.')
            return None 
        elif torque is None:
            herbpy.logger.error('Torque must be specified if stop_on_ft is true.')
            return None 
        elif len(force_direction) != 3:
            herbpy.logger.error('Force direction must be a three-dimensional vector.')
            return None
        elif len(torque) != 3:
            herbpy.logger.error('Torque must be a three-dimensional vector.')
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
    #TODO: Figure out better way to do this
    time.sleep(1.0)

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

    # Query the active manipulators based on which DOF indices are
    # included in the trajectory.
    active_manipulators = []
    config_spec = traj.GetConfigurationSpecification()
    group = config_spec.GetGroupFromName('joint_values')
    traj_indices = set([ int(index) for index in group.name.split()[2:] ])

    for manipulator in robot.manipulators:
        manipulator_indices = set(manipulator.GetArmIndices())
        if traj_indices & manipulator_indices:
            active_manipulators.append(manipulator)

    # Reset old trajectory execution flags
    for manipulator in active_manipulators:
        manipulator.ClearTrajectoryStatus()

    # Wait for the controller to finish execution.
    # TODO: Figure out why rendering trajectories fails on HERB.
    # TODO: Only wait for the relevant controllers.
    execution_done = False
    #with util.RenderTrajectory(robot, traj):
    robot.GetController().SetPath(traj)
    if timeout == None:
        execution_done = robot.WaitForController(0)
    elif timeout > 0:
        execution_done = robot.WaitForController(timeout)
        
    # Request the controller status from
    # each manipulator's controller.
    for manipulator in active_manipulators:
        status = manipulator.GetTrajectoryStatus()
        if status == 'aborted':
            raise exceptions.TrajectoryAborted('Trajectory aborted for %s' % manipulator.GetName())

    return traj

@HerbMethod
def WaitForObject(robot, obj_name, timeout=None, update_period=0.1):
    start = time.time()
    found_body = None

    robot.moped_sensorsystem.SendCommand('Enable')

    herbpy.logger.info("Waiting for object %s to appear.", obj_name)
    try:
        while True:
            # Check for an object with the appropriate name in the environment.
            bodies = robot.GetEnv().GetBodies()
            for body in bodies:
                if body.GetName().startswith('moped_' + obj_name):
                    return body

            # Check for a timeout.
            if timeout is not None and time.time() - start >= timeout:
                herbpy.logger.info("Timed out without finding object.")
                return None

            time.sleep(update_period)
    finally:
        robot.moped_sensorsystem.SendCommand('Disable')

@HerbMethod
def DriveStraightUntilForce(robot, direction, velocity=0.1, force_threshold=3.0,
                            max_distance=None, timeout=None, left_arm=True, right_arm=True):
    '''
    Drive the base in a direction until a force/torque sensor feels a force. The
    Segway first turns to face the desired direction, then drives forward at the
    specified velocity. The action terminates when max_distance is reached, the
    timeout is exceeded, or if a force is felt. The maximum distance and timeout
    can be disabled by setting the corresponding parameters to None.
    @param direction forward direction of motion in the world frame
    @param velocity desired forward velocity
    @param force_threshold threshold force in Newtons
    @param max_distance maximum distance in meters
    @param timeout maximum duration in seconds
    @param left_arm flag to use the left force/torque sensor
    @param right_arm flag to use the right force/torque sensor
    @return felt_force flag indicating whether the action felt a force
    '''
    if robot.segway_sim:
        raise Exception('DriveStraightUntilForce does not work with a simulated Segway.')
    elif (robot.left_ft_sim and left_arm) or (robot.right_ft_sim and right_arm):
        raise Exception('DriveStraightUntilForce does not work with simulated force/torque sensors.')

    env = robot.GetEnv()
    direction = numpy.array(direction, dtype='float')
    direction /= numpy.linalg.norm(direction) 
    manipulators = list()
    if left_arm:
        manipulators.append(robot.left_arm)
    if right_arm:
        manipulators.append(robot.right_arm)

    if not manipulators:
        herbpy.logger.warning('Executing DriveStraightUntilForce with no force/torque sensor for feedback.')

    # Tare the force/torque sensors.
    for manipulator in manipulators:
        manipulator.TareForceTorqueSensor()

    # Rotate to face the right direction.
    with env:
        robot_pose = robot.GetTransform()
    robot_angle = numpy.arctan2(robot_pose[1, 0], robot_pose[0, 0])
    desired_angle = numpy.arctan2(direction[1], direction[0])
    robot.RotateSegway(desired_angle - robot_angle)
    
    try:
        felt_force = False
        start_time = time.time()
        start_pos = robot_pose[0:3, 3]
        while True:
            # Check if we felt a force on any of the force/torque sensors.
            for manipulator in manipulators:
                force, torque = manipulator.GetForceTorque()
                if numpy.linalg.norm(force) > force_threshold:
                    return True

            # Check if we've exceeded the maximum distance.
            with env:
                current_pos = robot.GetTransform()[0:3, 3]
            distance = numpy.dot(current_pos - start_pos, direction)
            if max_distance is not None and distance >= max_distance:
                return False

            # Check for a timeout.
            time_now = time.time()
            if timeout is not None and time_now - star_time > timeout:
                return False

            # Continuously stream forward velocities.
            robot.segway_controller.SendCommand('DriveInstantaneous {0:f} 0 0'.format(velocity))
    finally:
        # Stop the Segway before returning.
        robot.segway_controller.SendCommand('DriveInstantaneous 0 0 0')

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
