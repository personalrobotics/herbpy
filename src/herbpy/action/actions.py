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
def GraspHandle(robot, pitcher):
    """
    @param robot The robot performing the handle grasp
    @param pitcher The pitcher who's handle will be grasped  
    """
    try:
        # Get the pose of the pitcher
        pitcher_in_world = pitcher.GetTransform()

        # Get the AABB of the pitcher
        pitcher_aabb = pitcher.ComputeAABB()

        # spout in pitcher
        spout_in_pitcher = numpy.array([[-0.7956, 0.6057, 0., -0.0662],
                                        [-0.6057, -0.7956, 0., -0.0504],
                                        [0., 0., 1., 0.2376], 
                                        [0., 0., 0., 1.]])

        # we want a hand pose orthogonal to the direction of the spout
        spout_direction = numpy.arctan2(spout_in_pitcher[0,1], spout_in_pitcher[0,0])
        palm_direction = spout_direction - 0.5*numpy.pi

        ee_in_pitcher = numpy.eye(4)
        ee_in_pitcher[:3,:3] = numpy.array([[0, 0, 1],
                                            [-1, 0, 0],
                                            [0, -1, 0]])
        ee_in_pitcher[:3,:3] = numpy.dot(ee_in_pitcher[:3,:3],
                                         openravepy.rotationMatrixFromAxisAngle([0, palm_direction, 0]))
                                         
        
        offset = pitcher_aabb.extents()[0] + 0.28 # pitcher radius + ee_offset
        ee_in_pitcher[:2,3] = -offset*ee_in_pitcher[:2,2]
        ee_in_pitcher[2,3] = 0.45*pitcher_aabb.extents()[2]

        ee_in_world = numpy.dot(pitcher_in_world, ee_in_pitcher)
        manip = robot.GetActiveManipulator()
        
        with prpy.viz.RenderPoses([ee_in_world], manip.GetRobot().GetEnv()):
            manip.PlanToEndEffectorPose(ee_in_world, execute=True)
        manip.PlanToEndEffectorOffset(direction=manip.GetEndEffectorTransform()[:3,2], distance=0.1, execute=True)
        manip.hand.CloseHand()
        robot.Grab(pitcher)

        return True
    except Exception, e:
        print 'GraspHandle planning failed: ', str(e)
        return False

@ActionMethod        
def Pour(robot, pitcher):
    """
    @param robot The robot performing the pouring
    @param pitcher The pitcher to perform the pouring  
    """
    # Now create a goal of tilting 
    min_tilt_amount = 95. #degrees
    max_tilt_amount = 100. #degrees
    success = False
    while not success and max_tilt_amount > 70.: 
        try:
            print 'Planning between %d and %d' % (min_tilt_amount, max_tilt_amount)
            manip = robot.GetActiveManipulator()
            manip.PlanToTSR(robot.tsrlibrary(pitcher, 'pour', 
                                             min_tilt = min_tilt_amount*numpy.pi/180., 
                                             max_tilt = max_tilt_amount*numpy.pi/180.),
                            smoothingitrs=100, execute=True)
            success = True 
        except Exception, e:
            print 'Pour planning failed: ', str(e)
            min_tilt_amount -= 5.
            max_tilt_amount -= 5.
            success = False
    return success, min_tilt, max_tilt

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