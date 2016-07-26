#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=C0103
# pylint: disable=E1101

'''
Testing script to get HERB in the kitchen.
'''

import numpy, prpy, herbpy, openravepy, IPython
from catkin.find_in_workspaces import find_in_workspaces

def getFridgePlanner(robot):
    '''Set up planning suite without trajopt'''
    from prpy.planning import Sequence, FirstSupported
    from prpy.planning import NamedPlanner, TSRPlanner

    actual_planner = Sequence(robot.snap_planner, robot.vectorfield_planner)
    planner = FirstSupported(Sequence(actual_planner,
                            TSRPlanner(delegate_planner=actual_planner),
                            robot.cbirrt_planner),
                            NamedPlanner(delegate_planner=actual_planner))
    return planner

def PushDoorOpen(robot, fridge):
    #NOTE this is entirely untested
    """
    @param robot
    @param fridge
    """
    planner = getFridgePlanner(robot)
    robot.base.Forward(0.1)

    # reshape the hand
    manip = robot.GetActiveManipulator()
    manip.hand.MoveHand(f1=0., f2=0., f3=0., spread=0)
 
    # grab fridge pose
    fridge_pose = fridge.GetTransform()

    # grab the location of the upper handle (lower handle is another link)
    upperHandle = fridge.GetLink('upper_handle')
    upperHandlePose = upperHandle.GetTransform()
 
    # aim for the middle of the door
    aabb = upperHandle.ComputeAABB()
    graspPose = upperHandlePose
    translationOffset = [-0.30, -0.075, -0.08]
    graspPose[0:3, 3] = graspPose[0:3, 3] + translationOffset + (aabb.pos() - fridge_pose[0:3, 3])
 
    # Rotate the pose so that it aligns with the correct hand pose
    # rotate with homogeneous transforms
    graspPose = graspPose.dot(openravepy.matrixFromAxisAngle([1, 0, 0], numpy.pi * 0.5))
    graspPose = graspPose.dot(openravepy.matrixFromAxisAngle([0, 1, 0], -numpy.pi * 0.5))
    graspPose = graspPose.dot(openravepy.matrixFromAxisAngle([0, 0, 1], numpy.pi * 1.5))

    slow_velocity_limits = numpy.array([0.17, 0.17, 0.475, 0.475, 0.625, 0.625, 0.625])
    manip.SetVelocityLimits(2.0*slow_velocity_limits, min_accel_time=0.2)
    manip.hand.MoveHand(0.2, 0.2, 0.2, 0)
    #return graspPose

    with prpy.rave.Disabled(fridge):
        #Localization movements
        #manip.PlanToEndEffectorPose(graspPose)
        planner.PlanToEndEffectorOffset(robot, graspPose)
        manip.SetVelocityLimits(slow_velocity_limits, min_accel_time=0.2)
        manip.MoveUntilTouch([1, 0, 0], 0.1)
        manip.PlanToEndEffectorOffset([-1, 0, 0], 0.02)
        manip.MoveUntilTouch([0, 0, -1], 0.15)
        manip.PlanToEndEffectorOffset([-1, -0.4, 0], 0.4)

        robot.base.Forward(-0.1)
    return graspPose
    
if __name__ == '__main__':
    env, robot = herbpy.initialize(sim=True)
    env.SetViewer('interactivemarker')
    robot.right_arm.SetActive()

    # PR kitchen
    kitchen_env = find_in_workspaces(
        search_dirs=['share'],
        project='pr_ordata',
        path='data/kitchen/pr_kitchen.env.xml',
        first_match_only=True)[0]
 
    try:
        env.Load(kitchen_env)
    except:
        print 'Fail'
    walls = env.GetKinBody('walls')
    lowercabinets = env.GetKinBody('lowercabinets')
    uppercabinets = env.GetKinBody('uppercabinets')
    fridge = env.GetKinBody('refrigerator')
    microwave = env.GetKinBody('microwave')
    dishwasher = env.GetKinBody('dishwasher')

    env.Remove(walls)
    env.Remove(lowercabinets) 
    env.Remove(uppercabinets)
    env.Remove(microwave)
    env.Remove(dishwasher)
     
    robot.DriveTo(fridge, planning=False)
    robot.GraspFridge(fridge)
    IPython.embed()
    # Then I run: robot.OpenHandle(fridge)
