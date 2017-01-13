import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain


@TSRFactory('herb', 'block', 'grasp')
def block_grasp(robot, block, manip=None, **kw_args):
    """
    Generates end-effector poses for moving the arm near the block
    @param robot The robot grasping the block
    @param block The block being grasped
    @param manip The manipulator to move near the block, if None
       the active manipulator of the robot is used
    """
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    offset = 0.01  #vertical offset relative to block
    alpha = 0.8  # orientation of end-effector relative to block

    block_in_world = block.GetTransform()
    block_in_world[:3, :3] = numpy.eye(3)  # ignore orientation
    ee_in_block = numpy.array([[
        numpy.cos(alpha), 0., -numpy.sin(alpha), 0.3 * numpy.sin(alpha) + 0.04
    ], [0., -1, 0, 0.
        ], [-numpy.sin(alpha), 0., -numpy.cos(alpha), 0.25 + offset],
                               [0., 0., 0., 1.]])
    Bw = numpy.zeros((6, 2))
    Bw[5, :] = [-numpy.pi, numpy.pi]

    pose_tsr = TSR(T0_w=block_in_world,
                   Tw_e=ee_in_block,
                   Bw=Bw,
                   manip=manip_idx)

    pose_tsr_chain = TSRChain(
        sample_start=False, sample_goal=True, TSRs=[pose_tsr])
    return [pose_tsr_chain]


@TSRFactory('herb', 'block', 'place')
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
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    # Block on the object
    T0_w = numpy.eye(4)
    T0_w[:3, 3] = position

    Bw = numpy.zeros((6, 2))
    Bw[5, :] = [-numpy.pi, numpy.pi]

    place_tsr = TSR(T0_w=T0_w, Tw_e=numpy.eye(4), Bw=Bw, manip=manip_idx)

    ee_in_block = numpy.dot(
        numpy.linalg.inv(block.GetTransform()),
        manip.GetEndEffectorTransform())
    ee_tsr = TSR(
        T0_w=numpy.eye(4),  #ignored
        Tw_e=ee_in_block,
        Bw=numpy.zeros((6, 2)),
        manip=manip_idx)

    place_tsr_chain = TSRChain(
        sample_start=False, sample_goal=True, TSRs=[place_tsr, ee_tsr])
    return [place_tsr_chain]


@TSRFactory('herb', 'block', 'place_on')
def block_on_surface(robot, block, pose_tsr_chain, manip=None):
    '''
    Generates end-effector poses for placing the block on a surface.
    This factory assumes the block is grasped at the time it is called.
    
    @param robot The robot grasping the block
    @param block The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses
                          for the block
    @param manip The manipulator grasping the object, if None the active
       manipulator of the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    block_pose = block.GetTransform()
    block_pose[:3, :3] = numpy.eye(3)  # ignore orientation
    ee_in_block = numpy.dot(
        numpy.linalg.inv(block_pose), manip.GetEndEffectorTransform())
    Bw = numpy.zeros((6, 2))
    Bw[2, :] = [0., 0.04]  # Allow some vertical movement

    for tsr in pose_tsr_chain.TSRs:
        if tsr.manipindex != manip_idx:
            raise ValueError(
                'pose_tsr_chain defined for a different manipulator.')

    grasp_tsr = TSR(Tw_e=ee_in_block, Bw=Bw, manip=manip_idx)
    all_tsrs = list(pose_tsr_chain.TSRs) + [grasp_tsr]
    place_chain = TSRChain(
        sample_start=False, sample_goal=True, constrain=False, TSRs=all_tsrs)

    return [place_chain]
