#!/usr/bin/env python
from prpy.planning.base import PlanningError
from herbpy.action.pushing import PushToPoseOnTable

import logging
logger = logging.getLogger(__name__)

if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser(
        description="Demonstrate using push planner to make object graspable")
    parser.add_argument(
        "-v",
        "--viewer",
        dest="viewer",
        default=None,
        help="The viewer to attach")
    parser.add_argument(
        "-d", "--debug", action="store_true", help="Run in debug mode")
    parser.add_argument(
        "-t",
        "--timeout",
        default=120.0,
        type=float,
        help="The max time to run the planner")
    args = parser.parse_args()

    import herbpy
    env, robot = herbpy.initialize(sim=True, attach_viewer=args.viewer)
    push_arm = robot.right_arm
    grasp_arm = robot.left_arm

    # Put the hand in a preshape for pushing
    push_arm.hand.MoveHand(spread=0, f1=0.75, f2=0.75, f3=0.75)

    # Table
    import os
    table_path = os.path.join('objects', 'table.kinbody.xml')
    table = env.ReadKinBodyXMLFile(table_path)

    import numpy, openravepy
    table_transform = numpy.eye(4)
    table_transform[:3, :3] = openravepy.rotationMatrixFromAxisAngle(
        [1.20919958, 1.20919958, 1.20919958])
    table_transform[:3, 3] = [0.65, 0.0, 0.0]
    table.SetTransform(table_transform)
    env.Add(table)

    # Glass
    glass_path = os.path.join('objects', 'plastic_glass.kinbody.xml')
    glass = env.ReadKinBodyXMLFile(glass_path)
    env.Add(glass)
    glass_transform = numpy.eye(4)
    glass_transform[:3, 3] = [0.6239455840637041, -0.4013916328109689, 0.75]
    glass.SetTransform(glass_transform)

    # Goal pose for the object
    import numpy
    goal_in_table = [0.15, 0., -0.03, 1.]
    goal_in_world = numpy.dot(table.GetTransform(), goal_in_table)
    goal_pose = goal_in_world[:2]
    goal_radius = 0.1

    from prpy.rave import Disabled
    from prpy.util import ComputeEnabledAABB
    with Disabled(table, padding_only=True):
        table_aabb = ComputeEnabledAABB(table)
    table_height = 0.01 + table_aabb.pos()[2] + table_aabb.extents()[2]

    pts = []
    for a in numpy.arange(0, 2. * numpy.pi, 0.1):
        pts.append([
            goal_pose[0] + goal_radius * numpy.sin(a),
            goal_pose[1] + goal_radius * numpy.cos(a), table_height
        ])
    h = env.drawlinestrip(
        points=numpy.array(pts),
        linewidth=5.0,
        colors=numpy.array([0.0, 1., 0.]))

    # Plan to the start configuration
    try:
        push_arm.SetActive()
        start_pose = [
            4.49119545, -1.59899798, -0.6, 1.65274406, -1.7742985, -0.63854765,
            -1.23051631
        ]
        push_arm.PlanToConfiguration(start_pose, execute=True)

        # Plan to push the object
        traj = robot.PushToPoseOnTable(
            obj=glass,
            table=table,
            goal_position=goal_pose,
            goal_radius=goal_radius,
            manip=push_arm,
            num_control_samples=1,
            max_plan_duration=args.timeout,
            debug=args.debug,
            render=True if args.viewer is not None else False)
        # Plan the arm home
        push_arm.PlanToNamedConfiguration('home', execute=True)

        # Plan to grasp with the other arm
        grasp_arm.SetActive()
        robot.PushGrasp(glass, manip=grasp_arm)

        # Plan that arm home with the object
        robot.Lift(glass)
        grasp_arm.PlanToNamedConfiguration('home', execute=True)

    except PlanningError as e:
        logger.error('Failed to complete task.')

    import IPython
    IPython.embed()
