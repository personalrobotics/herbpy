import herbpy
import openravepy
import numpy
import util
from planner import PlanningError 

WamMethod = util.CreateMethodListDecorator()

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
def OpenHand(manipulator, timeout=None):
    """
    Open the hand with a fixed spread.
    @param timeout blocking execution timeout
    """
    # TODO: Load this angle from somewhere.
    manipulator.MoveHand(f1=0.0, f2=0.0, f3=0.0, timeout=timeout)

@WamMethod
def CloseHand(manipulator, timeout=None):
    """
    Close the hand with a fixed spread.
    @param timeout blocking execution timeout
    """
    # TODO: Load this angle from somewhere.
    manipulator.MoveHand(f1=3.2, f2=3.2, f3=3.2, timeout=timeout)

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
    manipulator.ft_sensor.SendCommand('Tare')

@WamMethod
def SetVelocityLimits(manipulator, velocity_limits, min_accel_time):
    """
    Change the OWD velocity limits and acceleration time.
    @param velocity_limits individual joint velocity limits
    @param min_accel_time acceleration limit
    """
    num_dofs = len(manipulator.GetArmIndices())
    if len(velocity_limits) != num_dofs:
        herbpy.logging.error('Incorrect number of velocity limits; expected {0:d}, got {1:d}.'.format(
                             num_dofs, len(velocity_limits)))
        return False

    args  = [ 'SetSpeed' ]
    args += [ str(min_accel_time) ]
    args += [ str(velocity) for velocity in velocity_limits ]
    args_str = ' '.join(args)
    manipulator.arm_controller.SendCommand(args_str)
    return True

@WamMethod
def SetActive(manipulator):
    manipulator.parent.SetActiveManipulator(manipulator)

@WamMethod
def MoveUntilTouch(manipulator, direction, distance, max_force=5, execute=True):
    """
    Execute a straight move-until-touch action. This action stops when
    the maximum force is 
    @param direction unit vector for the direction o fmotion in the world frame
    @param distance maximum distance in meters
    @param max_force maximum force in Newtons
    @param execute optionally execute the trajectory
    @return traj output trajectory
    """
    # Compute the expected force direction in the hand frame.
    direction = numpy.array(direction)
    hand_pose = manipulator.GetEndEffectorTransform()
    force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

    # Plan a straight trajectory.
    traj = manipulator.PlanToEndEffectorOffset(direction, distance, execute=False)
    traj = manipulator.parent.AddTrajectoryFlags(traj, stop_on_ft=True, force_direction=force_direction,
                                          force_magnitude=max_force, torque=[100,100,100])

    if execute:
        manipulator.TareForceTorqueSensor()
        return manipulator.parent.ExecuteTrajectory(traj, retime=False)
    else:
        return traj
