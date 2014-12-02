import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *

@TSRFactory('herb', 'glass', 'grasp')
def glass_grasp(robot, glass, manip=None):
    '''
    @param robot The robot performing the grasp
    @param glass The glass to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = glass.GetTransform()
    Tw_e = numpy.array([[ 0., 0., 1., -0.225], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.08], 
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [-0.02, 0.02]  # Allow a little vertical movement
    Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    
    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, constrain=False, TSR = grasp_tsr)

    return [grasp_chain]

@TSRFactory('herb', 'glass', 'push_grasp')
def glass_push_grasp(robot, glass, manip=None):
    '''
    This factory differes from glass_grasp in that it places the manipulator 
    10 cm further away and assumes the manip will perform a push after
    moving to this TSR.  This allows for dealing with uncertainty in pose estimation of the
    object. 
    After using this code to move the end-effector into place, the robot
    should push in the direction of the z-axis of the manipulator:
      direction = manip.GetEndEffectorTransform()[:3,2]

    @param robot The robot performing the grasp
    @param glass The glass to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = glass.GetTransform()
    Tw_e = numpy.array([[ 0., 0., 1., -0.325], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.08], 
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [-0.02, 0.02]  # Allow a little vertical movement
    Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    
    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
                
@TSRFactory('herb', 'glass', 'place')
def glass_on_table(robot, glass, pose_tsr_chain, manip=None):
    '''
    Generates end-effector poses for placing the glass on the table.
    This factory assumes the glass is grasped at the time it is called.
    
    @param robot The robot grasping the glass
    @param glass The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses for the glass
    @param manip The manipulator grasping the object, if None the active
       manipulator of the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    ee_in_glass = numpy.dot(numpy.linalg.inv(glass.GetTransform()), manip.GetEndEffectorTransform())
    Bw = numpy.zeros((6,2))
    
    for tsr in pose_tsr_chain.TSRs:
        if tsr.manipindex != manip_idx:
            raise Exception('pose_tsr_chain defined for a different manipulator.')

    grasp_tsr = TSR(Tw_e = ee_in_glass, Bw = Bw, manip = manip_idx)
    all_tsrs = list(pose_tsr_chain.TSRs) + [grasp_tsr]
    place_chain = TSRChain(sample_start = False, sample_goal = True, constrain = False,
                           TSRs = all_tsrs)

    return  [ place_chain ]
    
@TSRFactory('herb', 'glass', 'transport')
def glass_transport(robot, glass, manip=None, roll_epsilon=0.2, pitch_epsilon=0.2, yaw_epsilon=0.2):
    '''
    Generates a trajectory-wide constraint for transporting the object with little roll, pitch or yaw
    Assumes the object has already been grasped and is in the proper
    configuration for transport.

    @param robot The robot grasping the glass
    @param glass The grasped object
    @param manip the manipulator grasping the object, if None the active manipulator 
       of the robot is used
    @param roll_epsilon The amount to let the object roll during transport (object frame)
    @param pitch_epsilon The amount to let the object pitch during transport (object frame)
    @param yaw_epsilon The amount to let the object yaw during transport (object frame)
    '''
   
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    ee_in_glass = numpy.dot(numpy.linalg.inv(glass.GetTransform()), manip.GetEndEffectorTransform())
    Bw = numpy.array([[-100., 100.], # bounds that cover full reachability of manip
                      [-100., 100.],
                      [-100., 100.],
                      [-roll_epsilon, roll_epsilon],
                      [-pitch_epsilon, pitch_epsilon],
                      [-yaw_epsilon, yaw_epsilon]])
    transport_tsr = TSR(T0_w = glass.GetTransform(),
                        Tw_e = ee_in_glass,
                        Bw = Bw,
                        manip = manip_idx)
    transport_chain = TSRChain(sample_start = False, sample_goal=False, constrain=True,
                               TSR = transport_tsr)
    
    return [ transport_chain ]
