import numpy, math, openravepy, time, threading
import herbpy, planner, exceptions, util
from pr_msgs.srv import AppletCommand

HeadMethod = util.CreateMethodListDecorator()

@HeadMethod
def FollowHand(head, traj, manipulator):
    robot = head.parent
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
        with robot.CreateRobotStateSaver():
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

# PD gains
kp = [8, 2]
kd = 0

previous_error = None
dead_zones = [0.04, 0.04]


def look_at_hand(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    target_dofs = robot.FindHeadDofs(target)
    # Get velocity and constrain
    velocity = target_dofs - robot.GetDOFValues(robot.head.GetArmIndices())
    velocity = constrain_velocity(kp*velocity)
    robot.head.Servo(velocity)

def deadzone_error(error):
    if abs(error[0]) < dead_zones[0]:
        error[0] = 0
    if abs(error[1]) < dead_zones[1]:
        error[1]= 0
    return error

def look_at_hand_pd(robot, manipulator):
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

def look_at_hand_traj(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    traj = robot.LookAt(target, execute=True)

def constrain_velocity(velocity):
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
         
def pd_loop(robot, manipulator, duration, rate):
    start_time = time()
    stop_time = start_time + duration
    while time() < stop_time:
        look_at_hand_pd(robot, manipulator)
        sleep(1.0/rate)


