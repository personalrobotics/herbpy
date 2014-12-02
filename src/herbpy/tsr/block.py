import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *

@TSRFactory('herb', 'block', 'place')
def block_on_object(robot, block, obj, manip=None):
    '''
    Generates end-effector poses for placing the block on another object
    
    @param robot The robot grasping the block
    @param block The block being grasped
    @param obj The object to place the block on
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

    obj_aabb = obj.ComputeAABB()
    
    # Block on the object
    Tw_e = numpy.eye(4)
    Tw_e[:3,:3] = block.GetTransform()[:3,:3]
    Tw_e[2,3] = obj_aabb.extents()[2] + 0.02
    Bw = numpy.zeros((6,2))
    Bw[5,:] = [-numpy.pi, numpy.pi]

    place_tsr = TSR(T0_w = obj.GetTransform(),
                    Tw_e = Tw_e, 
                    Bw = Bw,
                    manip = manip_idx)

    ee_in_block = numpy.dot(numpy.linalg.inv(block.GetTransform()), manip.GetEndEffectorTransform())
    ee_tsr = TSR(T0_w = numpy.eye(4), #ignored
                 Tw_e = ee_in_block,
                 Bw = numpy.zeros((6,2)),
                 manip = manip_idx)

    place_tsr_chain = TSRChain(sample_start=False, 
                               sample_goal = True,
                               TSRs = [place_tsr, ee_tsr])
    return [place_tsr_chain]
        
