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
from enum import Enum

class Status(Enum):
    Success = 0x00
    InvalidCommand = 0x01
    Reserved = 0x03
    InvalidParameter = 0x04
    InvalidChecksum = 0x05
    FlashEraseError = 0x07
    FlashProgramError = 0x08


class Command(Enum):
    GetAllAngles = 0xE1


class X3Inclinometer(object):
    MAX_ANGLE = math.radians((2 ** 32 - 1) / 1000)

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
        self.reset()

        assert self.connection
        request = struct.pack('BB', address, Command.GetAllAngles.value)
        self.connection.write(request)

        response_binary = self.connection.read(15)

        residual = sum(ord(d) for d in response_binary) % 256
        if residual != 0:
            raise ValueError('Checksum failed; {:d} != 0.'.format(residual))

        response = struct.unpack('>IIIHB', response_binary)
        angles = [ math.radians(angle / 1000) for angle in response[0:3] ]
        temperature = response[3] / 100

        return angles, temperature


def calibrate(manipulator, nominal_config, padding=0.05):
    ActiveDOF = openravepy.Robot.SaveParameters.ActiveDOF

    with robot.CreateRobotStateSaver(ActiveDOF):
        robot.SetActiveDOFs(manipulator.GetArmIndices())
        min_limit, max_limit = robot.GetActiveDOFLimits()

        # TODO: This is a hack to avoid hitting the floor/head. We should
        # change the DOF limits outside this function.
        min_limit[1] = -0.5
        max_limit[1] =  0.5
        max_limit[3] =  2.7
        min_limit[5] = -1.4
        max_limit[5] =  1.4

        assert (max_limit - min_limit > padding).all()

        for ijoint in xrange(manipulator.GetArmDOF()):
            print('J{:d}: Moving to nominal configuration'.format(ijoint + 1))
            robot.PlanToConfiguration(nominal_config)

            print('J{:d}: Moving to negative joint limit'.format(ijoint + 1))
            min_config = numpy.array(nominal_config)
            min_config[ijoint] = min_limit[ijoint] + padding
            robot.PlanToConfiguration(min_config)

            # TODO: Collect samples.

            print('J{:d}: Moving to positive joint limit'.format(ijoint + 1))
            max_config = numpy.array(nominal_config)
            max_config[ijoint] = max_limit[ijoint] - padding
            robot.PlanToConfiguration(max_config)


if __name__ == '__main__':
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
    robot.planner = prpy.planning.SnapPlanner()

    nominal_config = [ math.pi, 0., 0., 0., 0., 0., 0. ]

    import time
    time.sleep(0.1)

    for link in robot.GetLinks():
        if link.GetName().startswith('/left'):
            link.Enable(False)
            link.SetVisible(False)

    robot.right_arm.hand.CloseHand()
    calibrate(robot.right_arm, nominal_config)

    """
    with X3Inclinometer(port='/dev/ttyUSB0') as sensor:
        while True:
            angles, temperature = sensor.get_all_angles()
    """
