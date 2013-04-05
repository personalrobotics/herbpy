import herbpy
import openravepy
import numpy
import util
from planner import PlanningError 
import math
from time import sleep, time
import threading
import math, numpy, openravepy
import herbpy, exceptions, util

WamMethod = util.CreateMethodListDecorator()

@WamMethod
def LookAtHand(manipulator, **kw_args):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    return manipulator.parent.LookAt(target, **kw_args)
    

@WamMethod
def SetStiffness(manipulator, stiffness):
    """
    Set the WAM's stiffness. This enables or disables gravity compensation.
    @param stiffness value between 0.0 and 1.0
    """
    manipulator.arm_controller.SendCommand('SetStiffness {0:f}'.format(stiffness))

@WamMethod
def Servo(manipulator, velocities):
    """
    Servo with an instantaneous vector joint velocities.
    @param joint velocities
    """
    num_dof = len(manipulator.GetArmIndices())
    if len(velocities) != num_dof:
        raise ValueError('Incorrect number of joint velocities. Expected {0:d}; got {0:d}.'.format(
                         num_dof, len(velocities)))

    manipulator.arm_controller.SendCommand('Servo ' + ' '.join([ str(qdot) for qdot in velocities ]))


@WamMethod
def ServoTo(manipulator, target, duration, timeStep = 0.05, collisionChecking= True):
    """
    Servo's the WAM to the target taking the duration passed to it
    @param target dofs
    @param duration of the full servo
    @param timeStep
    @param collisionChecking
    """
    steps = int(math.ceil(duration/timeStep))
    original_dofs = manipulator.parent.GetDOFValues(manipulator.GetArmIndices())
    velocity = numpy.array(target-manipulator.parent.GetDOFValues(manipulator.GetArmIndices()))
    velocities = v/steps#[v/steps for v in velocity]
    inCollision = False 
    if collisionChecking==True:
        inCollision = manipulator.CollisionCheck(target)
    if inCollision == False:       
        for i in range(1,steps):
            manipulator.Servo(velocities)
            time.sleep(timeStep)
        manipulator.Servo([0] * len(manipulator.GetArmIndices()))
        new_dofs = manipulator.parent.GetDOFValues(manipulator.GetArmIndices())
        return True
    return False

@WamMethod
def ServoCollisionCheck(manipulator, end_dof):
    with manipulator.GetEnv():
        robot_saver = manipulator.parent.CreateRobotStateSaver()
        for i in range(1,steps):
            q = manipulator.parent.GetDOFValues(manipulator.GetArmIndices)
            q = q + velocity*timeStep
            self.herb.SetActiveDOFValues(q)
            if manipulator.GetEnv().CheckCollision(manipulator.parent):
                print'Servo motion unable to complete. Detected collision.'
                return True
            if manipulator.parent.CheckSelfCollision():
                print'Servo motion unable to complete. Detected self collision.'
                return True
    return False


@WamMethod
def MoveHand(manipulator, f1=None, f2=None, f3=None, spread=None, timeout=None):
    """
    Change the hand preshape. This function blocks until trajectory execution
    finishes. This can be changed by changing the timeout parameter to a
    maximum number of seconds. Pass zero to return instantantly.
    @param f1 finger 1 angle
    @param f2 finger 2 angle
    @param f3 finger 3 angle
    @param spread spread angle
    @param timeout blocking execution timeout
    """
    # Default any None's to the current DOF values.
    hand_indices = sorted(manipulator.GetChildDOFIndices())
    preshape = manipulator.parent.GetDOFValues(hand_indices)

    if f1     is not None: preshape[0] = f1
    if f2     is not None: preshape[1] = f2
    if f3     is not None: preshape[2] = f3
    if spread is not None: preshape[3] = spread

    manipulator.hand_controller.SetDesired(preshape)
    if timeout == None:
        manipulator.parent.WaitForController(0)
    elif timeout > 0:
        manipulator.parent.WaitForController(timeout)

@WamMethod
def OpenHand(manipulator, spread=None, timeout=None):
    """
    Open the hand with a fixed spread.
    @param spread hand spread
    @param timeout blocking execution timeout
    """
    # TODO: Load this angle from somewhere.
    manipulator.MoveHand(f1=0.0, f2=0.0, f3=0.0, spread=spread, timeout=timeout)

@WamMethod
def CloseHand(manipulator, spread=None, timeout=None):
    """
    Close the hand with a fixed spread.
    @param spread hand spread
    @param timeout blocking execution timeout
    """
    # TODO: Load this angle from somewhere.
    manipulator.MoveHand(f1=3.2, f2=3.2, f3=3.2, spread=spread, timeout=timeout)

@WamMethod
def GetForceTorque(manipulator):
    """
    Gets the most recent force/torque sensor reading in the hand frame.
    @return force,torque force/torque in the hand frame
    """
    sensor_data = manipulator.ft_sensor.GetSensorData()
    return sensor_data.force, sensor_data.torque

@WamMethod
def TareForceTorqueSensor(manipulator):
    """
    Tare the force/torque sensor. This is necessary before using the sensor
    whenever the arm configuration has changed.
    """
    if not manipulator.ft_simulated:
        manipulator.ft_sensor.SendCommand('Tare')

@WamMethod
def GetStrain(manipulator):
    """
    Gets the most recent strain sensor readings.
    @return a list of strain for each finger
    """

    strain = [0., 0., 0.]
    if not manipulator.hand_simulated:
        sensor_data = manipulator.handstate_sensor.GetSensorData()
        strain = sensor_data.force # This is because we are overriding the force/torque sensor datatype
    return strain

@WamMethod
def GetBreakaway(manipulator):
    """
    Gets the most recent breakaway readings for each finger
    @return a list of breakaway flags for each finger
    """
    
    breakaway = [False, False, False]
    if not manipulator.hand_simulated:
        sensor_data = manipulator.handstate_sensor.GetSensorData()
        breakaway = sensor_data.torque # This is because we are overriding the force/torque sensor datatype

    return breakaway


@WamMethod
def SetVelocityLimits(manipulator, velocity_limits, min_accel_time):
    """
    Change the OWD velocity limits and acceleration time.
    @param velocity_limits individual joint velocity limits
    @param min_accel_time acceleration limit
    """
    velocity_limits = numpy.array(velocity_limits, dtype='float')
    num_dofs = len(manipulator.GetArmIndices())
    if len(velocity_limits) != num_dofs:
        herbpy.logging.error('Incorrect number of velocity limits; expected {0:d}, got {1:d}.'.format(
                             num_dofs, len(velocity_limits)))
        return False

    # Update the OpenRAVE limits.
    with manipulator.parent.GetEnv():
        active_indices = manipulator.GetArmIndices()

        or_velocity_limits = manipulator.parent.GetDOFVelocityLimits()
        or_velocity_limits[active_indices] = velocity_limits
        manipulator.parent.SetDOFVelocityLimits(or_velocity_limits)

        or_accel_limits = manipulator.parent.GetDOFAccelerationLimits()
        or_accel_limits[active_indices] = velocity_limits / min_accel_time
        manipulator.parent.SetDOFAccelerationLimits(or_accel_limits)

    # Update the OWD limits.
    if not manipulator.arm_simulated:
        args  = [ 'SetSpeed' ]
        args += [ str(min_accel_time) ]
        args += [ str(velocity) for velocity in velocity_limits ]
        args_str = ' '.join(args)
        manipulator.arm_controller.SendCommand(args_str)
    return True

@WamMethod
def GetTrajectoryStatus(manipulator):
    '''
    Gets the status of the current (or previous) trajectory executed by the
    controller.
    '''
    if not manipulator.arm_simulated:
        return manipulator.arm_controller.SendCommand('GetStatus')
    else:
        if manipulator.arm_controller.IsDone():
            return 'done'
        else:
            return 'active'

@WamMethod
def ClearTrajectoryStatus(manipulator):
    '''
    Clears the current trajectory execution status.
    '''
    if not manipulator.arm_simulated:
        manipulator.arm_controller.SendCommand('ClearStatus')

@WamMethod
def SetActive(manipulator):
    '''
    Sets this as the active manipulator and updates the active DOF indices.
    '''
    manipulator.parent.SetActiveManipulator(manipulator)
    manipulator.parent.SetActiveDOFs(manipulator.GetArmIndices())

@WamMethod
def GetArmDOFValues(manipulator):
    '''
    Gets this manipulator's DOF values.
    ''' 
    return manipulator.parent.GetDOFValues(manipulator.GetArmIndices())

@WamMethod
def SetArmDOFValues(manipulator, dof_values):
    '''
    Sets this manipulator's DOF values.
    ''' 
    manipulator.parent.SetDOFValues(dof_values, manipulator.GetArmIndices())

@WamMethod
def MoveUntilTouch(manipulator, direction, distance, max_force=5, **kw_args):
    """
    Execute a straight move-until-touch action. This action stops when a
    sufficient force is is felt or the manipulator moves the maximum distance.
    @param direction unit vector for the direction of motion in the world frame
    @param distance maximum distance in meters
    @param max_force maximum force in Newtons
    @param execute optionally execute the trajectory
    @param **kw_args planner parameters
    @return felt_force flag indicating whether we felt a force.
    """
    # Compute the expected force direction in the hand frame.
    direction = numpy.array(direction)
    hand_pose = manipulator.GetEndEffectorTransform()
    force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

    # Plan a straight trajectory.
    traj = manipulator.PlanToEndEffectorOffset(direction, distance, execute=False, **kw_args)
    traj = manipulator.parent.AddTrajectoryFlags(traj, stop_on_ft=True, force_direction=force_direction,
                                                 force_magnitude=max_force, torque=[100,100,100])

    # TODO: Use a simulated force/torque sensor in simulation.
    try:
        manipulator.TareForceTorqueSensor()
        manipulator.parent.ExecuteTrajectory(traj, execute=True)
        return False
    # Trajectory is aborted by OWD because we felt a force.
    except exceptions.TrajectoryAborted:
        return True

@WamMethod
def StartServoSim(manipulator):
    """
    Starts a new thread with the ServoSim method
    """
    t = threading.Thread(target=ServoSim, args=[manipulator])
    t.start()

@WamMethod
def ServoSim(manipulator):
    """
    Simulates servo commands for the given manipulator
    """
    manipulator.servo_velocity = numpy.zeros(len(manipulator.GetArmIndices()))
    manipulator.servo_timestamp = 0
    timestep = 0.025
    time_start = time()
    while True:
        with manipulator.parent.GetEnv():
            # Check for timeout
            if abs(time_start - manipulator.servo_timestamp)>0.1:
                manipulator.servo_velocity = numpy.zeros(len(manipulator.GetArmIndices()))
                time_start = time()
            else:
                # Set robot's dof
                current_dofs = numpy.array(manipulator.parent.GetDOFValues(manipulator.GetArmIndices()))
                vel = numpy.array(manipulator.servo_velocity)
                dof_indices = manipulator.GetArmIndices()
                manipulator.parent.SetDOFValues(current_dofs + vel*timestep, dof_indices)
                # Reset time
                time_start = time()
        sleep(timestep)



@WamMethod
def PlanToNamedConfiguration(manipulator, name, **kw_args):
    config_inds = numpy.array(manipulator.parent.configs[name]['dofs'])
    config_vals = numpy.array(manipulator.parent.configs[name]['vals'])

    if len(config_inds) == 0:
        raise Exception('Failed to find named config: %s'%name)
    

    # TODO: Hacky. Can we do this better?
    # Need to parse out left and right manipulator indices
    # Also do we want to parse out head?
    arm_indices = numpy.array(manipulator.GetArmIndices())
    arm_indices.sort()

    arm_vals= []
    arm_inds = []


    for dof, val in zip(config_inds, config_vals):
        if dof in arm_indices:
            arm_inds.append(dof)
            arm_vals.append(val)

    traj = None
    if len(arm_inds) > 0:
        manipulator.parent.SetActiveDOFs(arm_inds)
        traj = manipulator.PlanToConfiguration(arm_vals, **kw_args)
   
    return traj

