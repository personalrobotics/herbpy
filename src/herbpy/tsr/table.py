import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain

@TSRFactory('herb', 'table', 'point_on')
def point_on(robot, table, manip=None):
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

@TSRFactory('herb', 'table', 'table_edge')
def table_edge(robot, table, **kwargs):
    '''
    This creates a TSR that allows you to sample poses from either
    long edge of the table.
    The poses will have x pointing into the table and z aligned with world z
    
    @param robot The robot (unused)
    @param table The table
    '''
    table_in_world = table.GetTransform()
    
    # Extents of the talbe
    xdim = 0.93175 # half extent
    ydim = 0.3675 # half extent
    zdim = 0.3805 # half extent

    # We want to create a set of two TSRs, one for each long edge of the table
    Bw_1 = numpy.zeros((6,2))
    Bw_1[0,:] = [-xdim, xdim]
    Tw_e1 = numpy.array([[ 0., 1., 0., 0.],
                         [ 0., 0., 1., 2.*ydim],
                         [ 1., 0., 0, -zdim], 
                         [ 0., 0., 0., 1.]])

    tsr1 = TSR(T0_w = table_in_world,
               Tw_e = Tw_e1,
               Bw = Bw_1)
    tsr1_chain = TSRChain(TSR = tsr1);

    Bw_2 = numpy.zeros((6,2))
    Bw_2[0,:] = [-xdim, xdim]
    Tw_e2 = numpy.array([[ 0.,-1., 0., 0.],
                         [ 0., 0., 1., 2.*ydim],
                         [-1., 0., 0,  zdim],
                         [ 0., 0., 0., 1.]])
    
    tsr2 = TSR(T0_w = table_in_world,
               Tw_e = Tw_e2,
               Bw = Bw_2)
    tsr2_chain = TSRChain(TSR = tsr2);

    return [tsr1_chain, tsr2_chain]
