# TODO fix the imports
import numpy
import prpy.tsr 

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'fuze_bottle', 'point')
def point_at_obj(robot, bottle, manip='right'):
    '''
    @param robot The robot performing the point
    @param bottle The bottle to point at
    @param manip The manipulator to point with. This must be the right arm 
    '''
    if manip.GetName() == 'right':
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()
    else:
        raise Exception("The pointing tsr is only defined for the right manipulator")
		
    # compute T_ow
    T0_w_0 = bottle.GetTransform()
    T0_w_1 = numpy.identity(4)

    # compute T_we. Tuned once and used forever onward 
    # Ensures pointer points at object with right arm
    # TODO need to add increased documentation once resolved 
    arm_start =  numpy.array([[ 0.07277, -0.71135,  0.69905,  0.58575],
                              [ 0.03360,  0.70226,  0.71111, -0.44275],
                              [-0.99678, -0.02825,  0.07501,  0.95686],
                              [ 0.     ,  0.     ,  0.     ,  1.     ]])
		
    T_base_obj = numpy.array([[1, 0, 0, 1.0],
                              [0, 1, 0, 0.0],
                              [0, 0, 1, 1.0],
                              [0, 0, 0, 1.0]])

    TW_e_0 = numpy.dot(numpy.linalg.inv(T_base_obj), arm_start)
    TW_e_1 = numpy.identity(4)

    # compute B_w
    Bw_0 = numpy.zeros((6, 2))
    Bw_0[3, :] = [-numpy.pi, numpy.pi]
    Bw_0[4, :] = [0, numpy.pi]
    Bw_0[5, :] = [-numpy.pi, numpy.pi]

    Bw_1 = numpy.zeros((6, 2))
    Bw_1[2, :] = [-0.75, 1.]
 
    T_0 = prpy.tsr.TSR(T0_w=T0_w_0, Tw_e=TW_e_0, Bw=Bw_0, manip=manip_idx)
    T_1 = prpy.tsr.TSR(T0_w=T0_w_1, Tw_e=TW_e_1, Bw=Bw_1, manip=manip_idx)

    chain = prpy.tsr.TSRChain(TSRs=[T_0, T_1], sample_goal=True, 
                     sample_start=False, constrain=False)

    return [chain]
