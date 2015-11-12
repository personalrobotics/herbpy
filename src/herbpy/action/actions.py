import numpy, openravepy, prpy, time
from prpy import tsr
from prpy.tsr import tsrlibrary
from prpy.tsr.tsr import *

@ActionMethod
def MoveObject(robot, direction = [0, 0, 1], distance = 0.1):
	"""
	@param robot The robot performing the push grasp
	@param direction The direction to move the object
	@param distance The distance to move the object
	"""
    try:
        manip = robot.GetActiveManipulator()
        manip.PlanToEndEffectorOffset(direction, distance, position_tolerance=0.1, execute=True)
        return True
    except Exception, e:
        print 'MoveObject planning failed: ', str(e)
        return False

@ActionMethod
def PushGraspCup(robot, cup, range):
    """
    @param robot The robot performing the push grasp 
    @param cup The cup to push grasp
    @param range The range for the yaw
    """
    try:
        manip = robot.GetActiveManipulator()
        manip.PlanToTSR(Cup_Grasp(robot, cup, manip, range), smoothingitrs=100, execute=True)
        manip.PlanToEndEffectorOffset(direction=manip.GetEndEffectorTransform()[:3,2], max_distance = 0.15, distance=0.01, execute=True)
        manip.hand.CloseHand()
        manip.GetRobot().Grab(cup)
        return True
    except Exception, e:
        print 'PushGraspCup planning failed: ', str(e)
        return False

def Cup_Grasp(robot, cup, manip, range, push_distance=0.0, **kw_args):
	"""
	@param robot The robot performing the push grasp
	@param cup The cup to push grasp
	@param manip The manipulator to perform the grasp with 
       (if None active manipulator is used)
    @param range The range for the yaw
    @param push_distance The distance to push before grasping
	"""
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = cup.GetTransform()
    T0_w[:3, :3] = robot.GetTransform()[:3, :3]
    ee_to_palm = 0.18
    palm_to_cup_center = .045
    total_offset = ee_to_palm + palm_to_cup_center + push_distance
    Tw_e = numpy.array([[ 0., 0., 1., -total_offset], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.08], # cup height
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0.03, 0.04]  # Allow a little vertical movement
    Bw[5,:] = range
    
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = prpy.tsr.TSRChain(sample_start=False, sample_goal = True, 
                                    constrain=False, TSR = grasp_tsr)
    return [grasp_chain]