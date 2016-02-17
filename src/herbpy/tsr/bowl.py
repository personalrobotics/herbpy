import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain

@TSRFactory('herb', 'plastic_bowl', 'grasp')
def bowl_grasp(robot, bowl, manip=None, **kw_args):
    '''
    @param robot The robot performing the grasp
    @param bowl The bowl to grasp
    @param manip The manipulator to perform the grasp, 
       if None the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        manip.GetRobot().SetActiveManipulator(manip)
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = bowl.GetTransform()
    Tw_e = numpy.array([[1.,  0.,  0., 0.08],
                        [0., -1.,  0., 0.],
                        [0.,  0., -1., 0.34],
                        [0.,  0.,  0., 1.]])
    Bw = numpy.zeros((6,2))
    Bw[2,:] = [-0.02, 0.02] # Allow a little verticle movement
    Bw[5,:] = [-numpy.pi, numpy.pi] # Allow any orientation

    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, 
                           constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
    
@TSRFactory('herb', 'plastic_bowl', 'place')
def bowl_on_table(robot, bowl, pose_tsr_chain, manip=None):
    '''
    Generates end-effector poses for placing the bowl on the table.
    This factory assumes the bowl is grasped at the time it is called.
    
    @param robot The robot grasping the bowl
    @param bowl The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses for the bowl
    @param manip The manipulator grasping the object, if None the active
       manipulator of the robot is used
    '''
   
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        manip.GetRobot().SetActiveManipulator(manip)
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    ee_in_bowl = numpy.dot(numpy.linalg.inv(bowl.GetTransform()), manip.GetEndEffectorTransform())
    Bw = numpy.zeros((6,2)) 
    Bw[2,:] = [0., 0.08]  # Allow some vertical movement
   
    for tsr in pose_tsr_chain.TSRs:
        if tsr.manipindex != manip_idx:
            raise Exception('pose_tsr_chain defined for a different manipulator.')

    grasp_tsr = TSR(Tw_e = ee_in_bowl, Bw = Bw, manip = manip_idx)
    all_tsrs = list(pose_tsr_chain.TSRs) + [grasp_tsr]
    place_chain = TSRChain(sample_start = False, sample_goal = True, constrain = False,
                           TSRs = all_tsrs)

    return  [ place_chain ]
