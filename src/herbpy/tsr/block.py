import numpy
import prpy.tsr 

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'block', 'place')
def block_at_pose(robot, block, position, manip=None):
    '''
    Generates end-effector poses for placing the block on another object
    
    @param robot The robot grasping the block
    @param block The block being grasped
    @param position The position to place the block [x,y,z]
    @param manip The manipulator grasping the object, if None the 
       active manipulator of the robot is used
    '''
    
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    # Block on the object
    T0_w = numpy.eye(4)
    T0_w[:3,3] = position

    Bw = numpy.zeros((6,2))
    Bw[5,:] = [-numpy.pi, numpy.pi]

    place_tsr = prpy.tsr.TSR(T0_w = T0_w,
                             Tw_e = numpy.eye(4),
                             Bw = Bw,
                             manip = manip_idx)

    ee_in_block = numpy.dot(numpy.linalg.inv(block.GetTransform()), manip.GetEndEffectorTransform())
    ee_tsr = prpy.tsr.TSR(T0_w = numpy.eye(4), #ignored
                          Tw_e = ee_in_block,
                          Bw = numpy.zeros((6,2)),
                          manip = manip_idx)

    place_tsr_chain = prpy.tsr.TSRChain(sample_start=False, 
                                        sample_goal = True,
                                        TSRs = [place_tsr, ee_tsr])
    return [place_tsr_chain]
        
