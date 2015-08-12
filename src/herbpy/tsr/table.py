import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *

@TSRFactory('herb', 'table', 'point_on')
def point_on(robot, table, padding=0.0, manip=None):
    '''
    This creates a TSR that allows you to sample poses on the table.
    The samples from this TSR should be used to find points for object placement.
    They are directly on the table, and thus not suitable as an end-effector pose.
    Grasp specific calculations are necessary to find a suitable end-effector pose.

    @param robot The robot performing the grasp
    @param pitcher The pitcher to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()
            
    T0_w = table.GetTransform()

    # The frame is set on the table such that the y-axis is normal to the table surface
    Tw_e = numpy.array([[ 1., 0., 0., 0. ], 
                        [0., 0., 1., 0.75], 
                        [0., -1., 0., 0.], 
                        [0., 0., 0., 1.]])
    Bw = numpy.zeros((6,2))
    Bw[0,:] = [-0.93+padding, 0.93-padding] # move along x and z directios to get any point on table
    Bw[2,:] = [-0.38+padding, 0.38-padding]
    Bw[4,:] = [-numpy.pi, numpy.pi] # allow any rotation around y - which is the axis normal to the table top
    
    table_top_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    table_top_chain = TSRChain(sample_start = False, sample_goal = True, constrain=False, 
                               TSR = table_top_tsr)
    return [table_top_chain]
