import numpy, openravepy
import prpy.tsr
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain
      
@TSRFactory('herb', 'rubbermaid_ice_guard_pitcher', 'push_grasp')
def pitcher_grasp(robot, pitcher, push_distance=0.1, manip=None):
    '''
    @param robot The robot performing the grasp
    @param pitcher The pitcher to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''
    
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = pitcher.GetTransform()
    spout_in_pitcher = numpy.array([[-0.7956, 0.6057, 0., -0.0662],
                                    [-0.6057, -0.7956, 0., -0.0504],
                                    [0., 0., 1., 0.2376], 
                                    [0., 0., 0., 1.]])

    # we want a hand pose orthogonal to the direction of the spout
    spout_direction = numpy.arctan2(spout_in_pitcher[0,1], spout_in_pitcher[0,0])
    palm_direction = spout_direction - 0.5*numpy.pi

    ee_in_pitcher = numpy.eye(4)
    ee_in_pitcher[:3,:3] = numpy.array([[0., 0., 1.],
                                        [-1., 0., 0.],
                                        [0., -1., 0.]])
    ee_in_pitcher[:3,:3] = numpy.dot(ee_in_pitcher[:3,:3],
                                         openravepy.rotationMatrixFromAxisAngle([0, palm_direction, 0]))
                                         
    
    pitcher_extents = [0.07877473,0.06568845,0.11882638]    
    offset = pitcher_extents[0] + 0.18 + push_distance # pitcher radius + ee_offset
    ee_in_pitcher[:2,3] = -offset*ee_in_pitcher[:2,2]
    ee_in_pitcher[2,3] = 0.45*pitcher_extents[2]

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [-0.01, 0.01]  # Allow a little vertical movement
    
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = ee_in_pitcher, Bw = Bw, manip = manip_idx)
    grasp_chain = prpy.tsr.TSRChain(sample_start=False, sample_goal = True, 
                                    constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
        
@TSRFactory('herb', 'rubbermaid_ice_guard_pitcher', 'pour')
def pitcher_pour(robot, pitcher, min_tilt = 1.4, max_tilt = 1.57, manip=None, grasp_transform = None, 
                 pitcher_pose = None):
    '''
    @param robot The robot whose active manipulator should be used to pour
    @param pitcher The pitcher to pour
    @param min_tilt The minimum amount to tilt the pitcher in the pouring motion
    @param max_tilt The maximum amount to tilt the pitcher in the pouring motion
    @param grasp_transform The pose of the end-effector in world frame when grasping the pitcher, 
       if this parameter is not provided, it is assumed the robot is currently 
       grasping the object and the current end-effector transform is used
    '''

    if manip is None:
        manip = robot.GetActiveManipulator()
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    if pitcher_pose is None:
        pitcher_pose = pitcher.GetTransform()
    pitcher_in_world = pitcher.GetTransform()

    # spout in pitcher
    spout_in_pitcher = numpy.array([[-0.7956, 0.6057, 0., 0.],
                                    [-0.6057, -0.7956, 0., 0.],
                                    [0., 0., 1., 0.0599], 
                                    [0., 0., 0., 1.]])
    spout_offset = 0.113
    spout_in_pitcher[:2,3] = spout_offset*spout_in_pitcher[:2,0]
    
    # end-effector relative to spout
    ee_in_world = manip.GetEndEffectorTransform()
    ee_in_pitcher = numpy.dot(numpy.linalg.inv(pitcher_in_world), ee_in_world)
    ee_in_spout = numpy.dot(numpy.linalg.inv(spout_in_pitcher), ee_in_pitcher)
    Bw_pour = numpy.zeros((6,2))
    Bw_pour[4,:] = [0, 2*numpy.pi ]
    Bw_pour[3,:] = [-10*numpy.pi/180, 10*numpy.pi/180]
        
    tsr_0 = TSR(T0_w = pitcher_pose,
                Tw_e = spout_in_pitcher,
                Bw = numpy.zeros((6,2)),
                manip = manip_idx)

    tsr_1_constraint = TSR(Tw_e = ee_in_spout,
                           Bw = Bw_pour,
                           manip = manip_idx)

    pour_chain = TSRChain(sample_start = False,
                          sample_goal = False,
                          constrain = True,
                          TSRs = [tsr_0, tsr_1_constraint])

    Bw_goal = numpy.zeros((6,2))
    Bw_goal[4,:] = [ min_tilt, max_tilt ]
    tsr_1_goal = TSR(Tw_e = ee_in_spout,
                     Bw = Bw_goal,
                     manip = manip_idx)
    pour_goal = TSRChain(sample_start = False,
                         sample_goal = True,
                         constrain = False,
                         TSRs = [tsr_0, tsr_1_goal])

    return [pour_chain, pour_goal], min_tilt, max_tilt
