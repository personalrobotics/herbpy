import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *

@TSRFactory('herb', 'plastic_plate', 'lift')
def plate_lift(robot, plate, manip=None, distance=0.1):
    '''
    This creates a TSR for lifting the plate a specified distance. 
    It is assumed that when called, the robot is grasping the plate

    @param robot The robot to perform the lift
    @param bottle The plate to lift
    @param manip The manipulator to lift 
    @param distance The distance to lift the plate
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

@TSRFactory('herb', 'plastic_plate', 'grasp')
def plate_grasp(robot, plate, manip=None):
    '''
    @param robot The robot performing the grasp
    @param plate The plate to grasp
    @param manip The manipulator to perform the grasp, 
       if None the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    plate_radius = plate.ComputeAABB().extents()[0]
    T0_w = plate.GetTransform()
    Tw_e = numpy.array([[0., -1.,  0., 0.],
                        [0.,  0., -1., plate_radius + 0.18], #radius plus end-effector offset
                        [1.,  0.,  0., 0.],
                        [0.,  0.,  0., 1.]])
    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0.0, 0.01] # Allow a little verticle movement
    Bw[5,:] = [-numpy.pi, numpy.pi] # Allow any point around the edge of the plate

    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
    
@TSRFactory('herb', 'plastic_plate', 'place')
def plate_on_table(robot, plate, pose_tsr_chain, manip=None):
    '''
    Generates end-effector poses for placing the plate on the table.
    This factory assumes the plate is grasped at the time it is called.
    
    @param robot The robot grasping the plate
    @param plate The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses for the plate
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

    ee_in_plate = numpy.dot(numpy.linalg.inv(plate.GetTransform()), manip.GetEndEffectorTransform())
    Bw = numpy.zeros((6,2))
    
    for tsr in pose_tsr_chain.TSRs:
        if tsr.manipindex != manip_idx:
            raise Exception('pose_tsr_chain defined for a different manipulator.')

    # we want the plate at 45 degrees and above the pose on the table
    palm_direction = manip.GetEndEffectorTransform()[:,2]  # palm points in the direction of the z-axis
    tilt_axis = [-palm_direction[1], palm_direction[0], 0] # orthogonal to palm direction
    tilt_amount = 30. * numpy.pi/180.

    # rotate to align the tilt axis with the x-axis
    axis_direction = numpy.arctan2(tilt_axis[1], tilt_axis[0])
    Rz = numpy.array([[numpy.cos(axis_direction), -numpy.sin(axis_direction), 0., 0.],
                      [numpy.sin(axis_direction),  numpy.cos(axis_direction), 0., 0.],
                      [0.                       ,  0.                       , 1., 0.],
                      [0.                       ,  0.                       , 0., 1.]])
    Rx = numpy.array([[1.,  0.                    ,  0.                    , 0.],
                      [0.,  numpy.cos(tilt_amount), -numpy.sin(tilt_amount), 0.],
                      [0.,  numpy.sin(tilt_amount),  numpy.cos(tilt_amount), 0.],
                      [0.,  0.                    ,  0.                    , 1.]])
    Rz_inv = numpy.array([[numpy.cos(-axis_direction), -numpy.sin(-axis_direction), 0., 0.],
                          [numpy.sin(-axis_direction),  numpy.cos(-axis_direction), 0., 0.],
                          [0.                        ,  0.                        , 1., 0.],
                          [0.                        ,  0.                        , 0., 1.]])
    tilted_plate_in_table = numpy.dot(Rz, numpy.dot(Rx, Rz_inv))

    # Define how far above the table to go
    plate_radius = plate.ComputeAABB().extents()[0]
    tilted_plate_in_table[2,3] = plate_radius * numpy.cos(tilt_amount) + 0.02
                                 
    #plate_tsr = TSR(Tw_e = tilted_plate_in_table, Bw = numpy.zeros((6,2)), manip = manip_idx)
    plate_in_table = numpy.eye(4)
    plate_in_table[2,3] = 0.20
    plate_tsr = TSR(Tw_e = plate_in_table, Bw = numpy.zeros((6,2)), manip = manip_idx)
    grasp_tsr = TSR(Tw_e = ee_in_plate, Bw = Bw, manip = manip_idx)
    all_tsrs = list(pose_tsr_chain.TSRs) + [plate_tsr, grasp_tsr]
    place_chain = TSRChain(sample_start = False, sample_goal = True, constrain = False,
                           TSRs = all_tsrs)

    return  [ place_chain ]
