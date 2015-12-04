#!/usr/bin/env python
import logging, numpy
from prpy.action import ActionMethod, ActionError
from prpy.planning.base import PlanningError
logger = logging.getLogger('herbpy')

@ActionMethod
def PushToPoseOnTable(robot, obj, table, goal_position, goal_radius, 
                      manip=None, max_plan_duration=30.0, 
                      movable_objects=None,
                      shortcut_time=3., render=True, bias=None,
                      print_stats=False, savedir=None,
                      **kw_args):
    """
    @param robot The robot performing the push
    @param obj The object to push
    @param table The table kinbody the object is resting on
    @param goal_position The desired (x,y) position of the object in world coordinates
    @param goal_radius The max distance from goal_position for the object to be to 
      still consider the goal achieved
    @param manip The manipulator to use for the push - if None the active manipulator is used
    @param max_plan_duration The max time to run the planner
    @param shortcut_time The amount of time to spend shortcutting, if 0. no shortcutting is performed
    @param render If true, render the trajectory while executing
    @param bias The bias factor to apply during tree extension, if None no bias used
    @param print_stats If true, print statistics about the planning instance after planning completes
    @param savedir If provided, write planner metadata and stats to directory
    """
    # Get a push planner
    try:
        from or_pushing.push_planner import PushPlanner
        planner = PushPlanner(robot.GetEnv())
    except ImportError:
        raise ActionError("Unable to create PushPlanner. Is the randomized_rearrangement_planning"
                          "repository checked out in your workspace?")

    # Get the manipulator
    if manip is None:
        with robot.GetEnv():
            manip = robot.GetActiveManipulator()


    with robot.GetEnv():
        from prpy.rave import Disabled
        from prpy.util import ComputeEnabledAABB
        with Disabled(table, padding_only=True):
            table_aabb = ComputeEnabledAABB(table)
        ee_pushing_transform = manip.GetEndEffectorTransform()
        goal_pose = obj.GetTransform()

    # Make the state bounds be at the edges of the table
    table_pos = table_aabb.pos()
    table_extents = table_aabb.extents()
    sbounds = {'high': [table_pos[0] + table_extents[0],
                        table_pos[1] + table_extents[1],
                        2.*numpy.pi],
               'low': [table_pos[0] - table_extents[0],
                       table_pos[1] - table_extents[1],
                       0]}
    
    # Assume we want to keep the current orientation and height of the manipulator
    #  throughout the push
    ee_pushing_transform[:2,3] = [0., 0.] #ignore x,y pose

    # Compute the goal pose
    table_height = table_pos[2] + table_extents[2]
    goal_pose[:3,3] = [goal_position[0],
                       goal_position[1],
                       table_height]

    # Add in parameters for running the bias RRT if desired
    if bias is not None:
        kw_args['bias'] = bias
        kw_args['bias_type'] = 'DivergenceBias'
        kw_args['bias_num'] = 4
        kw_args['bias_std'] = 0.01

    try:
        traj = None
        with robot.CreateRobotStateSaver():
            traj = planner.PushToPose(robot, obj, goal_pose,
                                      state_bounds = sbounds,
                                      movable_objects=movable_objects,
                                      pushing_manifold = ee_pushing_transform.flatten().tolist(),
                                      max_plan_duration = max_plan_duration,
                                      goal_epsilon = goal_radius,
                                      **kw_args)
    finally:
        if print_stats:
            # Print out statistics from planning instance
            stats = planner.GetLastPlanStats()
            print 'Planner stats: '
            for key in sorted(stats.keys()):
                print '\t%s - %s' % (key, stats[key])
        if savedir is not None:
            import os, yaml

            # Write planning data to file
            logger.info('Writing planning metadata to directory: %s' % savedir)
    
            if not os.path.exists(savedir):
                os.makedirs(savedir)

            path = planner.GetPlannedPath()
            if path is not None:
                pathfile = os.path.join(savedir, "path.ompl")
                with open(pathfile, 'w') as f:
                    f.write(yaml.dump(path))
            if traj is not None:
                trajfile = os.path.join(savedir, "path.traj")
                with open(trajfile, 'w') as f:
                    f.write(traj.serialize())
            stats = planner.GetLastPlanStats()
            statsfile = os.path.join(savedir, "planner.stats")
            with open(statsfile, 'w') as f:
                f.write(yaml.dump(stats))
            planner.WritePlannerData(savedir)

    # Execute
    from prpy.viz import RenderTrajectory
    with RenderTrajectory(robot, traj, color=[1, 0, 0, 1], render=render):
        if shortcut_time > 0:
            traj = planner.ShortcutPath(timelimit=shortcut_time)
        with RenderTrajectory(robot, traj, color=[0, 0, 1, 1], render=render):
          if manip.simulated:
              # Use the push planner code to simulate path execution.
              # This simulates the pushing of the objects during
              # execution of the trajectory.
              planner.ExecutePlannedPath()

          else:
              raw_input('Turn on camera and press enter to play trajectory')
              # Execute the trajectory
              robot.ExecuteTrajectory(traj)

              # During execution, object pushes won't be simulated. 
              # In the OpenRAVE world, the robot will instead move through the objects
              # and probably be in collision at the end.
              # This call sets all the objects to their expected poses
              # at the end of the trajectory. If execution was successful, this should resolve 
              # collisions. 
              planner.SetFinalObjectPoses()

    


    return traj
