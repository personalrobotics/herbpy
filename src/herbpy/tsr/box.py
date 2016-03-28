import numpy
import prpy.tsr

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'box', 'stamp')
def box_stamp(robot, box, manip=None):

    '''
    This creates a TSR for stamping a box. 
    It is assumed that when called, the robot is grasping a stamp

    @param robot The robot to perform the stamp
    @param box The box to stamp
    @param manip The manipulator to stamp 
    '''
    
    with robot.GetEnv():
        with robot.CreateRobotStateSaver():
            if manip is None:
                manip_idx = robot.GetActiveManipulatorIndex()
                manip = robot.GetActiveManipulator()
            else:
                manip.SetActive()
                manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

        T0_w = box.GetTransform()
    
    ee_to_palm = 0.18
    palm_to_box_center = .045
    adjustment = -0.09
    total_offset = ee_to_palm + palm_to_box_center + adjustment
    Tw_e = numpy.array([[ 0., 0., 1., -total_offset], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.38], # box height 0.2  0.35
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    #Bw[2,:] = [0.0, 0.00]  # Allow a little vertical movement
    Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = prpy.tsr.TSRChain(sample_start=False, sample_goal = True, 
                                    constrain=False, TSR = grasp_tsr)

    return [grasp_chain]



