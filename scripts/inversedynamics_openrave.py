#!/usr/bin/env python
"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""

import argparse, herbpy, logging, numpy, openravepy, sys


def iD():
    robot.right_arm.SetActive()
    jpos_list = [
        [ 3.1670137672513565, -1.5896855774682224, -0.030490730254705722, 1.5286675450710463, -0.032386115821109854, -0.03605547869739324, -0.026511110707008654 ],
        [ 5.147064052327087, -1.3622957134787639, -0.46951207447023835, 1.4621965585328196, -0.6525084419130015, 0.9042586427293137, -1.00526432576227 ],
        [ 4.121337486391367, 0.5703433599319374, -2.2175569328355724, 2.049783564999927, -0.2955432490571725, -0.646366247533136, -1.4040613050409545 ],
        [ 3.102831025390782, -0.04296665515610201, 0.1361467570187895, 0.045580105054784076, -3.0213214848284657, 0.0714728073293458, -0.24661498332101073 ]
        ]

    for i,jpos in enumerate(jpos_list):

        robot.SetActiveDOFValues(jpos, openravepy.KinBody.CheckLimitsAction.Nothing)
        robot.SetActiveDOFVelocities(numpy.zeros(7))

        jtor = robot.ComputeInverseDynamics([])
        print('jpos[{}] gravtorques: {}'.format(i, ' '.join(['{:7.3f}'.format(x) for x in jtor[robot.GetActiveDOFIndices()]])))


if __name__ == "__main__":

    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()

    # herbpy_args = {'sim':args.sim,
    #                'attach_viewer':args.viewer,
    #                'robot_xml':args.robot_xml,
    #                'env_path':args.env_xml,
    #                'segway_sim':args.segway_sim}
    # if args.sim and not args.segway_sim:
    #     herbpy_args['segway_sim'] = args.sim
    
    env, robot = herbpy.initialize(sim=True, segway_sim=True)
    iD()
