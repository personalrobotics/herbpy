#!/usr/bin/env python
from __future__ import division
from __future__ import print_function
import herbpy
import prpy.planning
import openravepy
import math
import numpy
import serial
import struct
import yaml
import tf.transformations
import rospy
from enum import IntEnum

ActiveDOF = openravepy.Robot.SaveParameters.ActiveDOF 


class Status(IntEnum):
    Success = 0x00
    InvalidCommand = 0x01
    Reserved = 0x03
    InvalidParameter = 0x04
    InvalidChecksum = 0x05
    FlashEraseError = 0x07
    FlashProgramError = 0x08


class Command(IntEnum):
    GetAllAngles = 0xE1
    SetOneDirection = 0xC4
    SetOneAngleOffset = 0xCF


class Direction(IntEnum):
    NORMAL = 0x00
    REVERSED = 0x01


class X3Inclinometer(object):
    """ Interface to the US Digital X3 Multi-Axis Absolute MEMS Inclinometer
    """
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.connection = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, type, value, traceback):
        self.disconnect()

    def connect(self):
        assert self.connection is None

        self.connection = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
        )

    def disconnect(self):
        assert self.connection is not None

        self.connection.close()

    def reset(self):
        self.connection.flushInput()
        self.connection.flushOutput()

    def get_all_angles(self, address=0x00):
        assert self.connection

        request = struct.pack('BB', address, Command.GetAllAngles.value)
        self.connection.write(request)

        response_binary = self.connection.read(15)
        self._checksum(response_binary)

        response = struct.unpack('>iiiHB', response_binary)
        angles = [ math.radians(angle / 1000) for angle in response[0:3] ]
        temperature = response[3] / 100

        return angles, temperature

    def set_one_direction(self, axis, direction, address=0):
        assert self.connection
        assert axis in [ 0, 1, 2 ]

        request = struct.pack('BBBB', address, Command.SetOneDirection.value, axis, direction.value)
        checksum = 256 - (self._sum_bytes(request) % 256)
        request += chr(checksum)

        self._checksum(request)
        self.connection.write(request)

        response_binary = self.connection.read(2)
        self._checksum(response_binary)

    def set_one_angle_offset(self, axis, offset, address=0):
        assert self.connection

        offset_raw = int(offset / 1000 + 0.5)
        assert axis in [ 0, 1, 2 ]
        assert -360000 <= offset_raw <= 359999

        request = struct.pack('>BBBi', address, Command.SetOneAngleOffset.value, axis, offset_raw)
        checksum = 256 - (self._sum_bytes(request) % 256)
        request += chr(checksum)

        self._checksum(request)
        self.connection.write(request)

        response_binary = self.connection.read(2)
        self._checksum(response_binary)

        response = struct.unpack('BB', response_binary)
        if response[0] != Status.Success.value:
            raise ValueError('Set failed: {:d}.'.format(response[0]))

    def _sum_bytes(self, data):
        return sum(ord(d) for d in data) 

    def _checksum(self, data):
        if not self._sum_bytes(data) % 256 == 0:
            raise ValueError('Checksum failed.')


def get_gravity_vector(vals):
   # compose a 3x3 matrix
   A = numpy.zeros((3,3))
   
   for i,val in enumerate(vals):
     
      # expresses atan2 relationships
      if i == 0:
         arg1 = (-1., 1)
         arg2 = ( 1., 2)
      elif i == 1:
         arg1 = (-1., 0)
         arg2 = (-1., 2)
      else: # i == 2
         arg1 = (-1., 1)
         arg2 = (-1., 0)
     
      # compute tangents
      tval = math.tan(val)
      cval = 1.0/math.tan(val) # better version?
      if abs(tval) < abs(cval):
         # use tangent
         pass
      else:
         # use cotangent
         tval = cval
         arg1, arg2 = arg2, arg1
     
      # fill matrix
      A[i,arg2[1]] = arg2[0] * tval
      A[i,arg1[1]] = - arg1[0]
   
   U, S, VT = numpy.linalg.svd(A)
   return tuple(VT[2,:])


# Axes:
# 2 - out of the palm, z-axis
joint_inclinometer_axes = [
    2, # J1
    1, # J2
    2, # J3
    1, # J4
    2, # J5
    1, # J6
    2, # J7
]

def calibrate(sensor, manipulator, nominal_config, ijoint, iaxis=None,
              padding=0.05, wait=5., smooth=True):
    min_limit, max_limit = robot.GetActiveDOFLimits()

    """
    if max_limit[ijoint] - min_limit[ijoint] > math.pi - padding:
        raise ValueError('Limits of J{:d} are too far apart.'.format(ijoint))
    """

    print('J{:d}: Moving to nominal configuration'.format(ijoint + 1))
    robot.PlanToConfiguration(nominal_config, smooth=smooth)

    print('J{:d}: Moving to negative joint limit: {:f}'.format(ijoint + 1, min_limit[ijoint] + padding))
    min_config = numpy.array(nominal_config)
    min_config[ijoint] = min_limit[ijoint] + padding
    robot.PlanToConfiguration(min_config, smooth=smooth)

    print('J{:d}: Collecting sample at negative joint limit'.format(ijoint + 1))
    time.sleep(wait)
    min_actual = robot.GetActiveDOFValues()[ijoint]
    min_measurement, _ = sensor.get_all_angles()

    print('J{:d}: Moving to positive joint limit: {:f}'.format(ijoint + 1, max_limit[ijoint] - padding))
    max_config = numpy.array(nominal_config)
    max_config[ijoint] = max_limit[ijoint] - padding
    robot.PlanToConfiguration(max_config, smooth=smooth)

    print('J{:d}: Collecting sample at positive joint limit'.format(ijoint + 1))
    time.sleep(wait)
    max_actual = robot.GetActiveDOFValues()[ijoint]
    max_measurement, _ = sensor.get_all_angles()

    angle_actual = max_actual - min_actual

    # Compute the angle between the two gravity vectors.
    if iaxis is None:
        assert max_limit[ijoint] - min_limit[ijoint] < math.pi - padding

        min_gravity = get_gravity_vector(min_measurement)
        max_gravity = get_gravity_vector(max_measurement)
        angle_measurement = math.acos(numpy.dot(min_gravity, max_gravity))
    # Use the specified axis of the inclinometer.
    else:
        angle_measurement = abs(max_measurement[iaxis] - min_measurement[iaxis])

        if angle_measurement - angle_actual > math.pi:
            angle_measurement = angle_measurement - 2 * math.pi
        elif angle_measurement - angle_actual < -math.pi:
            angle_measurement = angle_measurement + 2 * math.pi

    return angle_actual, angle_measurement

if __name__ == '__main__':
    sim = False
    namespace = '/right/owd/'
    output_path = 'transmission_ratios.yaml'

    env, robot = herbpy.initialize(sim=sim, segway_sim=True, vision_sim=True, head_sim=True, left_arm_sim=True, left_hand_sim=True, attach_viewer='interactivemarker')
    robot.planner = prpy.planning.SnapPlanner()

    # TODO: Hack to work around a race condition in or_interactivemarker.
    import time
    time.sleep(0.1)

    manipulator = robot.right_arm
    robot.SetActiveDOFs(manipulator.GetArmIndices())

    # Pad the joint limits to avoid hitting the floor and head.
    min_limit, max_limit = robot.GetDOFLimits()
    arm_indices = manipulator.GetArmIndices()
    min_limit[arm_indices[1]] = -0.5
    max_limit[arm_indices[1]] =  0.5
    max_limit[arm_indices[3]] =  1.5
    min_limit[arm_indices[4]] = -1.4
    max_limit[arm_indices[4]] =  1.4
    min_limit[arm_indices[5]] = -1.4
    max_limit[arm_indices[5]] =  1.4

    """
    for ijoint, dof_index in enumerate(arm_indices):
        residual = max_limit[dof_index] - min_limit[dof_index] - math.pi - 0.3
        if residual > 0:
            min_limit[dof_index] += residual / 2
            max_limit[dof_index] -= residual / 2
            print('Reduced range of J{:d} by {:f}.'.format(ijoint, residual))
    """

    robot.SetDOFLimits(min_limit, max_limit)

    nominal_config = [ math.pi, 0., 0., 0., 0., 0., 0. ]

    # Close the hand for safety.
    manipulator.hand.CloseHand()

    # TODO: Temporarily disable the left arm, since it isn't present on the
    # real robot. We'll need to disable this when we get the other arm back.
    for link in robot.GetLinks():
        if link.GetName().startswith('/left'):
            link.Enable(False)
            link.SetVisible(False)

    output_data = {}

    if not sim:
        rospy.init_node('transmission_ratio_calibrator', anonymous=True)

    with X3Inclinometer(port='/dev/ttyUSB0') as sensor:
        # Reset the inclinometer by setting all offsets to zero and reverting
        # to the default sign on all axes.
        print('X3: Clearing read/write buffers')
        sensor.reset()

        print('X3: Configuring sensor')
        for iaxis in range(3):
            sensor.set_one_angle_offset(iaxis, 0.)
            sensor.set_one_direction(iaxis, Direction.NORMAL)


        """
        while True:
            angles, _ = sensor.get_all_angles()
            print('{: 1.7f} {: 1.7f} {: 1.7f}'.format(*angles))

        raise Exception() 
        """

        # Sequentially calibrate each joint.
        for ijoint in xrange(manipulator.GetArmDOF()):
            print()
            print()

            # Compute the error in the current transmission ratio.
            angle_encoders, angle_sensor = calibrate(
                sensor, robot.right_arm, nominal_config,
                ijoint=ijoint, iaxis=joint_inclinometer_axes[ijoint]
            )

            # TODO: Is this flipped?
            correction = angle_encoders / angle_sensor

            print()
            print('J{:d}: Predicted:  {: 1.7f} radians'.format(ijoint + 1, angle_encoders))
            print('J{:d}: Measured:   {: 1.7f} radians'.format(ijoint + 1, angle_sensor))
            print('J{:d}: Correction: {: 1.7f}'.format(ijoint + 1, correction))

            # Compute the new transmission ratio.
            if not sim or True:
                param_name = 'motor{:d}_transmission_ratio'.format(ijoint + 1)
                old_transmission_ratio = rospy.get_param(namespace + param_name)
                new_transmission_ratio = old_transmission_ratio * correction
                output_data[param_name] = new_transmission_ratio.tolist()

                print('J{:d}: Transmission Ratio: {: 1.7f} -> {: 1.7f}'.format(
                    ijoint + 1, old_transmission_ratio, new_transmission_ratio))

    # TODO: What about the differentials? These correspond to parameters: 
    #   - /right/owd/differential3_ratio
    #   - /right/owd/differential6_ratio

    with open(output_path, 'w') as output_file:
        yaml.dump(output_data, output_file)
        print('Wrote output to: {:s}'.format(output_path))

