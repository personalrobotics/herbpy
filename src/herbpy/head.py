import logging, numpy, math, openravepy, time
import planner, exceptions, util, wam

logger = logging.getLogger('herbpy')

class Pantilt(wam.WAM):
    def FollowHand(head, traj, manipulator):
        robot = head.GetRobot()
        traj_config_spec = traj.GetConfigurationSpecification()
        head_config_spec = head.GetArmConfigurationSpecification()
        arm_indices = manipulator.GetArmIndices()
        head_indices = head.GetArmIndices()

        # Construct a path of head joint values that starts at the current
        # configuration and tracks the arm at each waypoint. Note that there
        # may be no IK solution at some waypoints.
        head_path = list()
        head_path.append(robot.GetDOFValues(head_indices))
        last_ik_index = 0

        with robot.GetEnv():
            with robot:
                for i in xrange(1, traj.GetNumWaypoints()):
                    traj_waypoint = traj.GetWaypoint(i)
                    arm_dof_values = traj_config_spec.ExtractJointValues(traj_waypoint, robot, arm_indices)
                    # Compute the position of the right arm through the FK.
                    manipulator.SetArmDOFValues(arm_dof_values)
                    hand_pose = manipulator.GetEndEffectorTransform()

                    # This will be None if there is no IK solution.
                    head_dof_values = robot.FindHeadDOFs(hand_pose[0:3, 3])
                    head_path.append(head_dof_values)
                    if head_dof_values is not None:
                        final_ik_index = i

        # Propagate the last successful IK solution to all following waypoints.
        # This lets us avoid some edge cases during interpolation.
        for i in xrange(final_ik_index + 1, traj.GetNumWaypoints()):
            head_path[i] = head_path[final_ik_index]

        # Interpolate to fill in IK failures. This is guaranteed to succeed because
        # the first and last waypoints are always valid.
        for i in xrange(1, traj.GetNumWaypoints()):
            # TODO: Fix timestamps on waypoints in MacTrajectory so we can properly
            # interpolate between waypoints.
            if head_path[i] is None:
                head_path[i] = head_path[i - 1]

        # Append the head DOFs to the input trajectory.
        merged_config_spec = traj_config_spec + head_config_spec
        openravepy.planningutils.ConvertTrajectorySpecification(traj, merged_config_spec)

        for i in xrange(0, traj.GetNumWaypoints()):
            waypoint = traj.GetWaypoint(i)
            merged_config_spec.InsertJointValues(waypoint, head_path[i], robot, head_indices, 0)
            traj.Insert(i, waypoint, True)

    def LookAt(head, target, **kw_args):
        """
        Look at a point in the world frame. This creates and, optionally executes,
        a two-waypoint trajectory that starts at the current configuration and
        moves to a goal configuration found through the neck's inverse kinematics.
        @param target point in the world frame.
        @param execute immediately execute the trajectory
        @return trajectory head trajectory
        """
        dof_values = head.FindIK(target)
        if dof_values is not None:
            return head.MoveHeadTo(dof_values, **kw_args)
        else:
            logger.error('There is no IK solution available.')

    def MoveHeadTo(head, target_dof_values, execute=True, **kw_args):
        # Update the controllers to get new joint values.
        robot = head.GetRobot()
        with robot.GetEnv():
            robot.GetController().SimulationStep(0)
            current_dof_values = head.GetDOFValues()

        config_spec = head.GetArmConfigurationSpecification()
        traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), 'GenericTrajectory')
        traj.Init(config_spec)
        traj.Insert(0, current_dof_values, config_spec)
        traj.Insert(1, target_dof_values, config_spec)

        # Optionally exeucute the trajectory.
        if execute:
            return head.GetRobot().ExecuteTrajectory(traj, **kw_args)
        else:
            return traj

    def FindIK(robot, target):
        ik_params = openravepy.IkParameterization(target, openravepy.IkParameterization.Type.Lookat3D)
        return robot.head.ik_database.manip.FindIKSolution(ik_params, 0)

# PD gains
kp = [8, 2]
kd = 0

previous_error = None
dead_zones = [0.04, 0.04]


def StitchArmHeadTrajectories(head, manipulator, traj_head, traj_arm):
    robot = head.parent
    traj_config_spec = traj.GetConfigurationSpecification()
    head_config_spec = head.GetArmConfigurationSpecification()
    arm_indices = manipulator.GetArmIndices()
    head_indices = head.GetArmIndices()


    # find which of the traj has the longest duration
    longest_duration = 0
    if traj_arm.GetDuration() > traj_head.GetDuration():
        longest_duration = traj_arm.GetDuration()
    else:
        longest_duration = traj_head.GetDuration()

    waypoints_arm = traj_arm.GetWaypoints(0, traj_arm.GetNumWaypoints())
    waypoints_head = traj_head.GetWaypoints(0, traj_head.GetNumWaypoints())

    # Create a list of all waypoint times
    waypoint_list = []
    for w in waypoints_arm:
        t = traj_arm.GetConfigurationSpecification().ExtractDeltaTime(w)
        waypoint_list.append((traj_arm.GetConfigurationSpecification().ExtractDeltaTime(w)*traj_arm.GetDuration())/longest_duration)
    for w in waypoints_head:
        waypoint_list.append((traj.GetConfigurationSpecification().ExtractDeltaTime(w)*traj_head.GetDuration())/longest_duration)
        #traj.ExtractDeltaTime(waypoint)

    # Remove duplicates from list of waypoint times
    waypoint_times = list(set(waypoint_list))
    # Sort list of waypoint times
    waypoint_times.sort()
    
    new_arm_waypoints = []
    new_head_waypoints = []
    # Resample both trajectories
    for t in waypoint_times:
        new_head_waypoints.insert(traj_head.Sample((t*traj_head.GetDuration())/longest_duration))
        new_arm_waypoints.insert(traj_arm.Sample((t*traj_arm.GetDuration())/longest_duration))


    # Create new trajectories with just joint dofs
    active_indices = numpy.append(robot.right_arm.GetArmIndices(),robot.head.GetArmIndices())
    robot.SetActiveDOFs(active_indices)

    c = robot.GetActiveConfigurationSpecification()
    c.AddDeltatimeGroup

    new_traj = RaveCreateTrajectory(env,'')  
    new_traj.Init(c)
    
    for a, h in zip(new_arm_waypoints,new_head_waypoints):
        dofvalues_arm = traj.GetConfigurationSpecification().ExtractJointValues(a,robot,robot.right_arm.GetArmIndices())
        dofvalues_head = traj.GetConfigurationSpecification().ExtractJointValues(h,robot,robot.head.GetArmIndices())
        
        dofvalues = dofvalues_arm + dofvalues_head
        
        new_traj.Insert(index,new_traj.GetNumWaypoints(), dofvalues, robot.GetActiveConfigurationSpecification())

    return new_traj

def LookAtHand(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    target_dofs = robot.FindHeadDofs(target)
    # Get velocity and constrain
    velocity = target_dofs - robot.GetDOFValues(robot.head.GetArmIndices())
    velocity = constrain_velocity(kp*velocity)
    robot.head.Servo(velocity)

def DeadzoneError(error):
    if abs(error[0]) < dead_zones[0]:
        error[0] = 0
    if abs(error[1]) < dead_zones[1]:
        error[1]= 0
    return error

def LookAtHandPD(robot, manipulator):
    global previous_error
    d_e = 0
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    target_dofs = robot.FindHeadDofs(target)
    # Find error for PD controller
    error = target_dofs - robot.GetDOFValues(robot.head.GetArmIndices())
    error = deadzone_error(error)
    if previous_error!=None:
        d_e = error - previous_error
    error_gain = numpy.array([kp[0]*error[0], kp[1]*error[1]])
    velocity = error_gain + kd*d_e
    velocity = constrain_velocity(velocity)
    robot.head.Servo(velocity)
    previous_error = error

def LookAtHandTraj(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    traj = robot.LookAt(target, execute=True)

def ConstrainVelocity(velocity):
    vel_limits = robot.GetDOFVelocityLimits(robot.head.GetArmIndices())
    if velocity[0] > vel_limits[0]:
        velocity[0] = vel_limits[0]
    if velocity[0] < (-1*vel_limits[0]):
        velocity[0] = -1*vel_limits[0]

    if velocity[1] > vel_limits[1]:
        velocity[1] = vel_limits[1]
    if velocity[1] < (-1*vel_limits[1]):
        velocity[1] = -1*vel_limits[1]
    return velocity
         
def PDLoop(robot, manipulator, duration, rate):
    start_time = time()
    stop_time = start_time + duration
    while time() < stop_time:
        look_at_hand_pd(robot, manipulator)
        sleep(1.0/rate)
