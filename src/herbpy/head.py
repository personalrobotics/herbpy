import herbpy
import openravepy
import numpy
import planner
import util
from time import *
import math
import threading

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


