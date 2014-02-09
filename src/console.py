#!/usr/bin/env python
"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""
import argparse, herbpy, logging, numpy, openravepy, sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='utility script for loading HerbPy')
    parser.add_argument('-s', '--sim', action='store_true',
                        help='simulation mode')
    parser.add_argument('-v', '--viewer', nargs='?', const=True,
                        help='attach a viewer of the specified type')
    parser.add_argument('--robot-xml', type=str,
                        help='robot XML file; defaults to herb_description')
    parser.add_argument('--env-xml', type=str,
                        help='environment XML file; defaults to an empty environment')
    parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
    args = parser.parse_args()

    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env, robot = herbpy.initialize(sim=args.sim, attach_viewer=args.viewer,
                                   robot_xml=args.robot_xml, env_path=args.env_xml)

    import IPython
    IPython.embed()
