import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain

@TSRFactory('herb', 'fuze_bottle', 'point')
def point_at_obj(robot, bottle, manip=None):
    """
    @param robot The robot performing the point
    @param bottle The bottle to point at
    @param manip The manipulator to point with. This must be the right arm. 
    """
    if manip is None:
        manip = robot.right_arm

    with manip.GetRobot():
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()
    
    if manip.GetName() != 'right':
        raise prpy.exceptions.PrPyException('Pointing is only defined on the right arm.')
		
    # compute T_ow
    T0_w_0 = bottle.GetTransform()
    T0_w_1 = numpy.identity(4)

    # compute T_we with respect to right arm.  
    TW_e_0 = numpy.array([[ 0.07277, -0.71135,  0.69905, -0.41425],
                          [ 0.0336 ,  0.70226,  0.71111, -0.44275],
                          [-0.99678, -0.02825,  0.07501, -0.04314],
                          [ 0.     ,  0.     ,  0.     ,  1.     ]])
  
    TW_e_1 = numpy.identity(4)

    # compute B_w
    Bw_0 = numpy.zeros((6, 2))
    Bw_0[3, :] = [-numpy.pi, numpy.pi]
    Bw_0[4, :] = [0, numpy.pi]
    Bw_0[5, :] = [-numpy.pi, numpy.pi]

    Bw_1 = numpy.zeros((6, 2))
    Bw_1[2, :] = [-0.5, 0.5]
 
    T_0 = TSR(T0_w=T0_w_0, Tw_e=TW_e_0, Bw=Bw_0, manip=manip_idx)
    T_1 = TSR(T0_w=T0_w_1, Tw_e=TW_e_1, Bw=Bw_1, manip=manip_idx)

    chain = TSRChain(TSRs=[T_0, T_1], sample_goal=True, 
                     sample_start=False, constrain=False)
 
    return [chain]

@TSRFactory('herb', 'fuze_bottle', 'present')
def present_obj(robot, bottle, manip=None):
    """
    @param robot The robot performing the presentation gesture
    @param bottle The bottle to present
    @param manip The manipulator to present. This must be the right arm. 
    """

    if manip is None:
        manip = robot.right_arm

    with manip.GetRobot():
        manip.SetActive()
        manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    if manip.GetName() != 'right':
        raise prpy.exceptions.PrpyException('Presenting is only defined for the right arm.')

    #Compute T0_w
    T0_w = bottle.GetTransform()

    #Compute TW_e with respect to right arm
    TW_e = numpy.array([[-0.42480713,  0.81591405,  0.39220297, -0.10103751],
                        [ 0.08541525, -0.39518032,  0.91462383, -0.2806636 ],
                        [ 0.90124533,  0.42203884,  0.09818392,  0.16018878],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]])

    #Compute Bw
    Bw = numpy.zeros((6, 2))
    Bw[5, :] = [-numpy.pi, numpy.pi]

    T = TSR(T0_w=T0_w, Tw_e=TW_e, Bw=Bw, manip=manip_idx)
    chain = TSRChain(TSRs=[T], sample_goal=True, sample_start=False, 
            constrain=False)

    return [chain]

@TSRFactory('herb', 'fuze_bottle', 'lift')
def fuze_lift(robot, bottle, manip=None, distance=0.1):
    '''
    This creates a TSR for lifting the bottle a specified distance. 
    It is assumed that when called, the robot is grasping the bottle

    @param robot The robot to perform the lift
    @param bottle The bottle to lift
    @param manip The manipulator to lift 
    @param distance The distance to lift the bottle
    '''
    print 'distance = %0.2f' % distance

    if manip is None:
        manip = robot.GetActiveManipulator()
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
         with manip.GetRobot():
             manip.SetActive()
             manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    #TSR for the goal
    start_position = manip.GetEndEffectorTransform()
    end_position = manip.GetEndEffectorTransform()
    end_position[2, 3] += distance

    Bw = numpy.zeros((6, 2))
    epsilon = 0.05
    Bw[0,:] = [-epsilon, epsilon]
    Bw[1,:] = [-epsilon, epsilon]
    Bw[4,:] = [-epsilon, epsilon]

    tsr_goal = TSR(T0_w = end_position, Tw_e = numpy.eye(4),
            Bw = Bw, manip = manip_idx)

    goal_tsr_chain = TSRChain(sample_start = False, sample_goal = True, 
            constrain = False, TSRs = [tsr_goal])

    #TSR that constrains the movement
    Bw_constrain = numpy.zeros((6, 2))
    Bw_constrain[:, 0] = -epsilon
    Bw_constrain[:, 1] = epsilon
    if distance < 0:
        Bw_constrain[1,:] = [-epsilon+distance, epsilon]
    else:
        Bw_constrain[1,:] = [-epsilon, epsilon+distance]

    tsr_constraint = TSR(T0_w = start_position, Tw_e = numpy.eye(4),
            Bw = Bw_constrain, manip = manip_idx)

    movement_chain = TSRChain(sample_start = False, sample_goal = False, 
            constrain = True, TSRs = [tsr_constraint])

    return [goal_tsr_chain, movement_chain]


@TSRFactory('herb', 'fuze_bottle', 'grasp')
def fuze_grasp(robot, fuze, manip=None):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _fuze_grasp(robot, fuze, manip = manip)

@TSRFactory('herb', 'fuze_bottle', 'push_grasp')
def fuze_grasp(robot, fuze, push_distance = 0.1, manip=None):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _fuze_grasp(robot, fuze, push_distance = push_distance, manip = manip)

def _fuze_grasp(robot, fuze, push_distance = 0.0, manip = None):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = fuze.GetTransform()
    ee_to_palm_distance = 0.18
    default_offset_distance = 0.05 # This is the radius of the fuze
                                   # plus a little bit
    total_offset = ee_to_palm_distance + default_offset_distance + push_distance
    Tw_e = numpy.array([[ 0., 0., 1., -total_offset], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.108], # half of fuze bottle height
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0.0, 0.02]  # Allow a little vertical movement
    Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    
    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, constrain=False, TSR = grasp_tsr)

    return [grasp_chain]

