import logging, prpy
from prpy.action import ActionMethod

logger = logging.getLogger('herbpy')

@ActionMethod
def Grasp(robot, obj, manip=None, preshape=[0., 0., 0., 0.], **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param preshape The grasp preshape for the hand
    """
    HerbGrasp(robot, obj,  manip=manip, preshape=preshape)

@ActionMethod
def PushGrasp(robot, obj, distance=0.1, manip=None, preshape=[0., 0., 0., 0.], push_required=True, **kw_args):
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
    """
    HerbGrasp(robot, obj,  manip=manip, preshape=preshape, distance=distance)

def HerbGrasp(robot, obj, distance=None, manip=None, preshape=[0., 0., 0., 0.], push_required=False,
              **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param distance The distance to push before grasping (if None, no pushing)
    @param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param push_required If true, throw exception if a plan for the pushing 
       movement cannot be found. If false, continue with grasp even if push 
       cannot be executed. (only used if distance is not None)
    @param preshape The grasp preshape for the hand
    """
    if manip is None:
        manip = robot.GetActiveManipulator()

    # Move the hand to the grasp preshape
    manip.hand.MoveHand(f1 = preshape[0],
                        f2 = preshape[1],
                        f3 = preshape[2],
                        spread = preshape[3])

    # Get the grasp tsr
    grasp_tsr_list = robot.tsrlibrary(obj, 'grasp') 
    
    # Plan to the grasp
    with prpy.viz.RenderTSRList(grasp_tsr_list, robot.GetEnv()):
        manip.PlanToTSR(grasp_tsr_list)

    if distance is not None:
        # Manipulator must be active for grab to work properly
        active_manip = robot.GetActiveManipulator()
        robot.SetActiveManipulator(manip)
        robot.Grab(obj)
        robot.SetActiveManipulator(active_manip)
                
        # Now execute the straight line movement
        ee_in_world = manip.GetEndEffectorTransform()
        push_direction = ee_in_world[:3,2]
        with prpy.viz.RenderVector(ee_in_world[:3,3], push_direction,
                                   distance, robot.GetEnv()):
            try:
                manip.PlanToEndEffectorOffset(direction = push_direction,
                                              distance = distance,
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
    active_manip = robot.GetActiveManipulator()
    robot.SetActiveManipulator(manip)
    robot.Grab(obj)
    robot.SetActiveManipulator(active_manip)

