import herbpy, numpy, openravepy, util

HandMethod = util.CreateMethodListDecorator()

@HandMethod
def GetIndices(hand):
    return numpy.array(sorted(hand.manipulator.GetChildDOFIndices()))

@HandMethod
def GetDOFValues(hand):
    return hand.robot.GetDOFValues(hand.GetIndices())

@HandMethod
def SetDOFValues(hand, dof_values):
    hand.robot.SetDOFValues(dof_values, hand.GetIndices())

@HandMethod
def LookAt(hand, **kw_args):
    target = hand.GetTransform()[0:3, 3]
    return hand.robot.LookAt(target, **kw_args)

@HandMethod
def MoveHand(hand, f1=None, f2=None, f3=None, spread=None, timeout=None):
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
    preshape = hand.GetDOFValues()
    if f1     is not None: preshape[0] = f1
    if f2     is not None: preshape[1] = f2
    if f3     is not None: preshape[2] = f3
    if spread is not None: preshape[3] = spread

    hand.controller.SetDesired(preshape)
    if timeout == None:
        hand.robot.WaitForController(0)
    elif timeout > 0:
        hand.robot.WaitForController(timeout)

@HandMethod
def OpenHand(hand, spread=None, timeout=None):
    """
    Open the hand with a fixed spread.
    @param spread hand spread
    @param timeout blocking execution timeout
    """
    # TODO: Load this angle from somewhere.
    hand.MoveHand(f1=0.0, f2=0.0, f3=0.0, spread=spread, timeout=timeout)

@HandMethod
def CloseHand(hand, spread=None, timeout=None):
    """
    Close the hand with a fixed spread.
    @param spread hand spread
    @param timeout blocking execution timeout
    """
    # TODO: Load this angle from somewhere.
    hand.MoveHand(f1=3.2, f2=3.2, f3=3.2, spread=spread, timeout=timeout)

@HandMethod
def GetStrain(hand):
    """
    Gets the most recent strain sensor readings.
    @return a list of strain for each finger
    """
    if not hand.simulated:
        # This is because we are overriding the force/torque sensor datatype
        sensor_data = hand.handstate_sensor.GetSensorData()
        return sensor_data.force.copy()
    else:
        return numpy.zeros(3)

@HandMethod
def GetBreakaway(hand):
    """
    Gets the most recent breakaway readings for each finger
    @return a list of breakaway flags for each finger
    """
    if not hand.simulated:
        # This is because we are overriding the force/torque sensor datatype.
        sensor_data = hand.handstate_sensor.GetSensorData()
        breakaway = sensor_data.torque 
    else:
        return [ False, False, False ]

@HandMethod
def GetForceTorque(hand):
    """
    Gets the most recent force/torque sensor reading in the hand frame.
    @return force,torque force/torque in the hand frame
    """
    if not hand.ft_simulated:
        sensor_data = hand.ft_sensor.GetSensorData()
        return sensor_data.force, sensor_data.torque
    else:
        return numpy.zeros(3), numpy.zeros(3)

@HandMethod
def TareForceTorqueSensor(hand):
    """
    Tare the force/torque sensor. This is necessary before using the sensor
    whenever the arm configuration has changed.
    """
    if not hand.ft_simulated:
        hand.ft_sensor.SendCommand('Tare')
        sleep(2)
