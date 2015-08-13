import numpy
import prpy.tsr

@TSRFactory('herb', 'pop_tarts', 'lift')
def poptarts_lift(robot, pop_tarts, manip=None, distance=0.1):
    '''
    This creates a TSR for lifting the pop tarts a specified distance. 
    It is assumed that when called, the robot is grasping the pop tarts

    @param robot The robot to perform the lift
    @param pop_tarts The pop tarts box to lift
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

    tsr_goal = prpy.tsr.TSR(T0_w = end_position, Tw_e = numpy.eye(4),
            Bw = Bw, manip = manip_idx)

    goal_tsr_chain = prpy.tsr.TSRChain(sample_start = False, sample_goal = True, 
            constrain = False, TSRs = [tsr_goal])

    #TSR that constrains the movement
    Bw_constrain = numpy.zeros((6, 2))
    Bw_constrain[:, 0] = -epsilon
    Bw_constrain[:, 1] = epsilon
    if distance < 0:
        Bw_constrain[1,:] = [-epsilon+distance, epsilon]
    else:
        Bw_constrain[1,:] = [-epsilon, epsilon+distance]

    tsr_constraint = prpy.tsr.TSR(T0_w = start_position, Tw_e = numpy.eye(4),
            Bw = Bw_constrain, manip = manip_idx)

    movement_chain = prpy.tsr.TSRChain(sample_start = False, sample_goal = False, 
            constrain = True, TSRs = [tsr_constraint])

    return [goal_tsr_chain, movement_chain]


@TSRFactory('herb', 'pop_tarts', 'grasp')
def poptarts_grasp(robot, pop_tarts, manip=None):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop tarts box to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _poptarts_grasp(robot, pop_tarts, manip = manip)

@TSRFactory('herb', 'pop_tarts', 'push_grasp')
def poptarts_grasp(robot, pop_tarts, push_distance = 0.1, manip=None):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop_tarts to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _poptarts_grasp(robot, pop_tarts, push_distance = push_distance, manip = manip)

def _poptarts_grasp(robot, pop_tarts, push_distance = 0.0, manip = None):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop tarts box to grasp
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

    T0_w = pop_tarts.GetTransform()
    ee_to_palm_distance = 0.18
    default_offset_distance = 0.04 # This is the radius of the box
                                   # plus a little bit
    total_offset = ee_to_palm_distance + default_offset_distance + push_distance
    Tw_e = numpy.array([[ 0., 0., 1., -total_offset], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.08], # half of box height
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0.0, 0.02]  # Allow a little vertical movement
    Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = prpy.tsr.TSRChain(sample_start=False, sample_goal = True, 
            constrain=False, TSR = grasp_tsr)

    return [grasp_chain]

