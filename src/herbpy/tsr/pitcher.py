import numpy
import prpy.tsr

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'rubbermaid_ice_guard_pitcher', 'grasp')
def pitcher_grasp(robot, pitcher, manip=None):
    '''
    @param robot The robot performing the grasp
    @param pitcher The pitcher to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''
    
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = pitcher.GetTransform()
    Tw_e = numpy.array([[0.802, 0., -0.596, 0.199], 
                        [-0.5961, 0., -0.8028, 0.2684], 
                        [0., 1., 0., 0.1841], 
                        [0., 0., 0., 1.]])
    Bw = numpy.zeros((6,2))
    Bw[2,:] = [-0.01, 0.01]  # Allow a little vertical movement
    
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = prpy.tsr.TSRChain(sample_start=False, sample_goal = True, 
                                    constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
        
@prpy.tsr.tsrlibrary.TSRFactory('herb', 'rubbermaid_ice_guard_pitcher', 'pour')
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
        with manip.GetRobot():
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
    Bw_pour[4,:] = [ -0.2, 1.57 ]
        
    tsr_0 = prpy.tsr.TSR(T0_w = pitcher_pose,
                         Tw_e = spout_in_pitcher,
                         Bw = numpy.zeros((6,2)),
                         manip = manip_idx)

    tsr_1_constraint = prpy.tsr.TSR(Tw_e = ee_in_spout,
                                    Bw = Bw_pour,
                                    manip = manip_idx)

    pour_chain = prpy.tsr.TSRChain(sample_start = False,
                                   sample_goal = False,
                                   constrain = True,
                                   TSRs = [tsr_0, tsr_1_constraint])

    Bw_goal = numpy.zeros((6,2))
    Bw_goal[4,:] = [ min_tilt, max_tilt ]
    tsr_1_goal = prpy.tsr.TSR(Tw_e = ee_in_spout,
                              Bw = Bw_goal,
                              manip = manip_idx)
    pour_goal = prpy.tsr.TSRChain(sample_start = False,
                                  sample_goal = True,
                                  constrain = False,
                                  TSRs = [tsr_0, tsr_1_goal])

    return [pour_chain, pour_goal], min_tilt, max_tilt
