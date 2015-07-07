#!/usr/bin/env python
import logging
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
logger = logging.getLogger('herbpy')

@ActionMethod
def PushToPoseOnTable(robot, obj, table, goal_position, goal_radius, 
                      manip=None, max_plan_duration=30.0, **kw_args):
    """
    @param robot The robot performing the push
    @param obj The object to push
    @param table The table kinbody the object is resting on
    @param goal_position The desired (x,y) position of the object in world coordinates
    @param goal_radius The max distance from goal_position for the object to be to 
      still consider the goal achieved
    @param manip The manipulator to use for the push - if None the active manipulator is used
    @param max_plan_duration The max time to run the planner
    """
    # Get a push planner
    from or_pushing.push_planner import PushPlanner
    planner = PushPlanner(robot.GetEnv())

    # Get the manipulator
    if manip is None:
        with robot.GetEnv():
            manip = robot.GetActiveManipulator()

    # Make the state bounds be at the edges of the table
    with robot.GetEnv():
        table_aabb = table.ComputeAABB()
    table_pos = table_aabb.pos()
    table_extents = table_aabb.extents()
    sbounds = {'high': [table_pos[0] + table_extents[0],
                        table_pos[1] + table_extents[1],
                        2.*numpy.pi],
               'low': [table_pos[0] - table_extents[0],
                       table_pos[1] - table_extents[1],
                       0]}
    
    # This should be the pose and the height of the manipulator
    # for pushing - parallel to table, 0.1m above surface  
    ee_pushing_transform = numpy.array([[ 0.,  0., 1., 0], 
                                        [-1.,  0., 0., 0], 
                                        [ 0., -1., 0., table_pos[2] + table_extents[2] + 0.1], 
                                        [ 0.,  0., 0., 1]])

    # Compute the goal pose
    with robot.GetEnv():
        goal_pose = obj.GetTransform()
    table_height = table_pos[2] + table_extents[2] + 0.01
    goal_pose[:3,3] = [goal_position[0],
                       goal_position[1],
                       table_height]

    with robot.CreateRobotStateSaver():
        traj = planner.PushToPose(robot, obj, goal_pose,
                                  state_bounds = sbounds,
                                  pushing_manifold = ee_pushing_transform.flatten().tolist(),
                                  max_plan_duration = max_plan_duration,
                                  goal_epsilon = goal_radius,
                                  **kw_args)
    if traj is None:
        raise PlanningError('Failed to find pushing plan')

    # Execute
    if manip.simulated:
        planner.ExecutePlannedPath()
    else:
        robot.ExecuteTrajectory(traj)

    return traj
