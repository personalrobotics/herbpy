import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain

@TSRFactory('herb', 'prkitchen_refrigerator', 'open')
def fridge_open(robot, fridge, manip=None, minopen=0, maxopen=None, **kw_args):
    #add optional argument for which door (upper or lower)
    '''
    @param robot The robot performing the grasp
    @param fridge The fridge who's doors handle the robot will grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    @param minopen Minimum amount to open fridge
    @param maxopen Maximum amount to open fridge
    '''
  
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    if maxopen is None:
        maxopen = minopen

    # Get the hinge
    hinge = fridge.GetManipulator('door_lower_hinge')
    fridge.SetActiveManipulator(hinge)

    # Put T0_w of the first link at the world pose of the hinge
    hinge_in_world = hinge.GetTransform()
    ee_in_world = manip.GetEndEffectorTransform()

    handle_in_hinge = numpy.eye(4)
    handle_in_hinge[0:3, 3] = [-0.048, -0.643, -0.721]
    handle_in_world = numpy.dot(hinge_in_world, handle_in_hinge)
    hinge_in_world = numpy.dot(handle_in_world, numpy.linalg.inv(handle_in_hinge))

    # Construct the first TSR
    Bw1 = numpy.zeros((6, 2))
    Bw1[5, :] = [0., numpy.pi]
    tsr1 = TSR(T0_w=hinge_in_world, Tw_e=handle_in_hinge, Bw=Bw1, manip=manip_idx)

    # Now construct the second TSR for grasping the door.
    ee_in_handle = numpy.dot(numpy.linalg.inv(handle_in_world),
                            ee_in_world)
    Bw2 = numpy.zeros((6, 2))
    Bw2[5, :] = [-0.1, 0.1]
    #  Note:T0_w on this second tsr is ignored by planner
    tsr2 = TSR(T0_w=handle_in_world, Tw_e=ee_in_handle, Bw=Bw2, manip=manip_idx)

    # Now glue them together into a constraint chain
    tsrchain_constrain = TSRChain(TSRs=[tsr1, tsr2], constrain=True, sample_start=False, sample_goal=False, mimicbodyname='refrigerator', mimicbodyjoints=numpy.mat(0))

    # Modify the Bw matrix for the hinge for the min and max parameters
    Bw3 = numpy.zeros((6, 2))
    Bw3[5, :] = [minopen, maxopen]

    tsr1_goal = TSR(T0_w=hinge_in_world, Tw_e=handle_in_hinge, Bw=Bw3, manip=manip_idx)
    tsrchain_goal = TSRChain(TSRs=[tsr1_goal, tsr2], constrain=False, 
                             sample_start=False, sample_goal=True) #, 
                             #mimicbodyname='refrigerator', mimicbodyjoints=numpy.mat(0))

    return [tsrchain_goal, tsrchain_constrain]

