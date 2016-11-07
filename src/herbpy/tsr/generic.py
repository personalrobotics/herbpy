import numpy
from tsr.tsrlibrary import TSRFactory
from tsr.tsr import TSR, TSRChain
from prpy.util import GetManipulatorIndex

@TSRFactory('herb', None, 'point')
def point_obj(robot, transform, manip=None):
    """
    @param robot The robot performing the point
    @param transform The location of where the robot is pointing to
    @param manip The manipulator to point with. This must be the right arm. 
    """

    (manip, manip_idx) = GetManipulatorIndex(robot, manip)

    if manip.GetName() != 'right':
        raise prpy.exceptions.PrPyException('Pointing is only defined on the right arm.')
		
    # compute T_ow
    T0_w_0 = transform
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

@TSRFactory('herb', None, 'present')
def present_obj(robot, transform, manip=None):
    """
    @param robot The robot performing the presentation gesture
    @param transform The location of where the robot the presenting
    @param manip The manipulator to present. This must be the right arm. 
    """

    (manip, manip_idx) = GetManipulatorIndex(robot, manip)

    if manip.GetName() != 'right':
        raise prpy.exceptions.PrpyException('Presenting is only defined for the right arm.')

    #Compute T0_w
    T0_w = transform

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

@TSRFactory('herb', None, 'sweep')
def sweep_objs(robot, transform, manip=None):
    """
    This motion sweeps from where the manipulator currently is
    to the transform given as input. 

    @param robot The robot performing the point
    @param transform The location of end of the sweeping motion
    @param manip The manipulator to sweep.
    """

    (manip, manip_idx) = GetManipulatorIndex(robot, manip)

    #TSR for the goal
    ee_offset = 0.2
    obj_position = transform
    start_position = manip.GetEndEffectorTransform()
    end_position = manip.GetEndEffectorTransform()
    end_position[0, 3] = obj_position[0, 3]
    end_position[1, 3] = obj_position[1, 3] - ee_offset

    Bw = numpy.zeros((6, 2))
    epsilon = 0.05
    Bw[0,:] = [-epsilon, epsilon]
    Bw[1,:] = [-epsilon, epsilon]
    Bw[2,:] = [-epsilon, epsilon]
    Bw[4,:] = [-epsilon, epsilon]

    tsr_goal = TSR(T0_w = end_position, Tw_e = numpy.eye(4),
            Bw = Bw, manip = manip_idx)

    goal_tsr_chain = TSRChain(sample_start = False, sample_goal = True,
            constrain = False, TSRs = [tsr_goal])

    goal_in_start = numpy.dot(numpy.linalg.inv(start_position), end_position)
    
    #TSR that constrains the movement
    Bw_constrain = numpy.zeros((6, 2))
    Bw_constrain[:, 0] = -epsilon
    Bw_constrain[:, 1] = epsilon
    if goal_in_start[0,3] < 0:
        Bw_constrain[0,:] = [-epsilon+goal_in_start[0,3], epsilon]
    else:
        Bw_constrain[0,:] = [-epsilon, epsilon+goal_in_start[0,3]]

    if goal_in_start[1,3] < 0:
        Bw_constrain[1,:] = [-epsilon+goal_in_start[1,3], epsilon]
    else:
        Bw_constrain[1,:] = [-epsilon, epsilon+goal_in_start[1,3]]
    
    if goal_in_start[2,3] < 0:
        Bw_constrain[2,:] = [-epsilon+goal_in_start[2,3], epsilon]
    else:
        Bw_constrain[2,:] = [-epsilon, epsilon+goal_in_start[2,3]]

    tsr_constraint = TSR(T0_w = start_position, Tw_e = numpy.eye(4),
            Bw = Bw_constrain, manip = manip_idx)

    movement_chain = TSRChain(sample_start = False, sample_goal = False,
            constrain = True, TSRs = [tsr_constraint])

    return [goal_tsr_chain, movement_chain]

@TSRFactory('herb', None, 'lift')
def lift_obj(robot, transform=numpy.eye(4), manip=None, distance=0.1, epsilon=0.05):
    """
    This creates a TSR for lifting an object a specified distance. 
    It is assumed that when called, the robot is grasping the object.
    This assumes that the object can be lifted with one arm. 

    @param robot The robot to perform the lift
    @param transform The transform of the object to lift
    @param manip The manipulator to lift 
    @param distance The distance to lift the bottle
    """

    if manip is None:
        manip = robot.GetActiveManipulator()
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
         manip.SetActive()
         manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    #TSR for the goal
    start_position = manip.GetEndEffectorTransform()
    end_position = manip.GetEndEffectorTransform()
    end_position[2, 3] += distance

    Bw = numpy.zeros((6, 2))
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
