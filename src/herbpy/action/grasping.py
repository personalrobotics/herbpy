import logging, openravepy, prpy
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
from contextlib import contextmanager

logger = logging.getLogger('herbpy')

@ActionMethod
def Grasp(robot, obj, manip=None, preshape=[0., 0., 0., 0.], 
          tsrlist=None, render=True, **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param preshape The grasp preshape for the hand
    @param tsrlist A list of TSRChain objects to use for planning to grasp pose
       (if None, the 'grasp' tsr from tsrlibrary is used)
    @param render Render tsr samples and push direction vectors during planning
    """
    HerbGrasp(robot, obj,  manip=manip, preshape=preshape, 
              tsrlist=tsrlist, render=render)

@ActionMethod
def PushGrasp(robot, obj, push_distance=0.1, manip=None, 
              preshape=[0., 0., 0., 0.], push_required=True, 
              tsrlist=None, render=True, **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param distance The distance to push before grasping
    @param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param push_required If true, throw exception if a plan for the pushing 
       movement cannot be found. If false, continue with grasp even if push 
       cannot be executed.
    @param preshape The grasp preshape for the hand
    @param tsrlist A list of TSRChain objects to use for planning to grasp pose
       (if None, the 'grasp' tsr from tsrlibrary is used)
    @param render Render tsr samples and push direction vectors during planning
    """
    if tsrlist is None:
        tsrlist = robot.tsrlibrary(obj, 'push_grasp', push_distance=push_distance)

    HerbGrasp(robot, obj, manip=manip, preshape=preshape, 
              push_distance=push_distance,
              tsrlist=tsrlist, render=render)

def HerbGrasp(robot, obj, push_distance=None, manip=None, 
              preshape=[0., 0., 0., 0.], 
              push_required=False, 
              tsrlist=None,
              render=True,
              **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param distance The distance to push before grasping (if None, no pushing)
    @param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param preshape The grasp preshape for the hand
    @param push_required If true, throw exception if a plan for the pushing 
       movement cannot be found. If false, continue with grasp even if push 
       cannot be executed. (only used if distance is not None)
    @param render Render tsr samples and push direction vectors during planning
    """
    if manip is None:
        with robot.GetEnv():
            manip = robot.GetActiveManipulator()

    # Move the hand to the grasp preshape
    manip.hand.MoveHand(*preshape)

    # Get the grasp tsr
    if tsrlist is None:
        tsrlist = robot.tsrlibrary(obj, 'grasp')
    
    # Plan to the grasp
    with prpy.viz.RenderTSRList(tsrlist, robot.GetEnv(), render=render):
        manip.PlanToTSR(tsrlist, execute=True)

    if push_distance is not None:
        ee_in_world = manip.GetEndEffectorTransform()
        push_direction = ee_in_world[:3,2]

        # Move the object into the hand
        env = robot.GetEnv()
        with env:
            obj_in_world = obj.GetTransform()

            # First move back until collision
            stepsize = 0.01
            total_distance = 0.0
            while not env.CheckCollision(robot, obj) and total_distance <= push_distance:
                obj_in_world[:3,3] -= stepsize*push_direction
                total_distance += stepsize
                obj.SetTransform(obj_in_world)
            
            # Then move forward until just out of collision
            stepsize = 0.001
            while env.CheckCollision(robot, obj):
                obj_in_world[:3,3] += stepsize*push_direction
                obj.SetTransform(obj_in_world)

        # Manipulator must be active for grab to work properly
        p = openravepy.KinBody.SaveParameters
        with robot.CreateRobotStateSaver(p.ActiveManipulator):
            robot.SetActiveManipulator(manip)
            robot.Grab(obj)

                
        # Now execute the straight line movement
        with prpy.viz.RenderVector(ee_in_world[:3,3], push_direction,
                                   push_distance, robot.GetEnv(), render=render):
            try:
                with prpy.rave.Disabled(obj):
                    manip.PlanToEndEffectorOffset(direction = push_direction,
                                                  distance = push_distance,
                                                  execute = True,
                                                  **kw_args)
            except PlanningError, e:
                if push_required:
                    raise
                else:
                    logger.warn('Could not find a plan for straight line push. Ignoring.')

        robot.Release(obj)

    # Now close the hand to grasp
    manip.hand.CloseHand()

    # Manipulator must be active for grab to work properly
    p = openravepy.KinBody.SaveParameters
    with robot.CreateRobotStateSaver(p.ActiveManipulator):
        robot.SetActiveManipulator(manip)
        robot.Grab(obj)

@ActionMethod
def Lift(robot, obj, distance=0.05, manip=None, render=True, **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to lift
    @param distance The distance to lift the cup
    @param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param render Render tsr samples and push direction vectors during planning
    """
    if manip is None:
        with robot.GetEnv():
            manip = robot.GetActiveManipulator()

    # Check for collision and disable anything in collision
    creport = openravepy.CollisionReport()    
    disabled_objects = []

    # Resolve inconsistencies in grabbed objects
    if robot.CheckSelfCollision():
        grabbed_objs = robot.GetGrabbed()
        for obj in grabbed_objs:
            robot.Release(obj)
        for obj in grabbed_objs:
            robot.Grab(obj)

    # Create list of any current collisions so those can be disabled
    while robot.GetEnv().CheckCollision(robot, creport):
        collision_obj = creport.plink2.GetParent()
        disabled_objects.append(collision_obj)
        collision_obj.Enable(False)
        
    for obj in disabled_objects:
        obj.Enable(True)

    # Perform the lift
    with prpy.rave.AllDisabled(robot.GetEnv(), disabled_objects):
        lift_direction = [0., 0., 1.]
        lift_distance = distance
        ee_in_world = manip.GetEndEffectorTransform()
        with prpy.viz.RenderVector(ee_in_world[:3,3], lift_direction,
                                   distance, robot.GetEnv(), render=render):
            manip.PlanToEndEffectorOffset(direction=lift_direction,
                                          distance=lift_distance,
                                          execute=True,
                                          **kw_args)

@ActionMethod
def Place(robot, obj, on_obj, manip=None, render=True, **kw_args):
    """
    Place an object onto another object
    This assumes the 'point_on' tsr is defined for the on_obj and
    the 'place' tsr is defined for obj
    @param robot The robot performing the push grasp
    @param obj The object to place
    @param on_obj The object to place obj on
    @param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param render Render tsr samples and push direction vectors during planning
    """

    if manip is None:
        with robot.GetEnv():
            manip = robot.GetActiveManipulator()

    # Get a tsr to sample places to put the glass
    obj_extents = obj.ComputeAABB().extents()
    obj_radius = max(obj_extents[0], obj_extents[1])
    tray_top_tsr = robot.tsrlibrary(on_obj, 'point_on', padding=obj_radius)

    #  Now use this to get a tsr for sampling ee_poses
    place_tsr = robot.tsrlibrary(obj, 'place', pose_tsr_chain = tray_top_tsr[0])

    # Plan to the grasp
    with prpy.viz.RenderTSRList(place_tsr, robot.GetEnv(), render=render):
        manip.PlanToTSR(place_tsr, execute=True)

    # Open the hand
    manip.hand.OpenHand()

    # Release the object
    robot.Release(obj)
