import numpy, prpy
import prpy.tsr

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'block_bin', 'point_on')
def point_on(robot, block_bin, manip=None, padding=0.04):
    '''
    This creates a TSR that allows you to sample poses on the tray.
    The samples from this TSR should be used to find points for object placement.
    They are directly on the tray, and thus not suitable as an end-effector pose.
    Grasp specific calculations are necessary to find a suitable end-effector pose.

    @param robot The robot performing the grasp
    @param tray The tray to sample poses on
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    @param padding The amount of space around the edge to exclude from sampling
       If using this to place an object, this would be the maximum radius of the object
    @param handle_padding If true add extra padding along the edges of the tray that
       have the handles to prevent choosing a pose too near the handle of the tray
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()
            
    T0_w = block_bin.GetTransform() # Coordinate system on bottom of bin

    Tw_e = numpy.eye(4)
    Tw_e[2,3] = 0.17 # set the object on top of the bin - bin is 13cm high

    Bw = numpy.zeros((6,2))

    xdim = max(0.085 - padding, 0.0)
    ydim = max(0.135 - padding, 0.0)
    Bw[0,:] = [-xdim, xdim ] # move along x and y directions to get any point on tray
    Bw[1,:] = [-ydim, ydim]
    Bw[2,:] = [-0.02, 0.04] # verticle movement
    Bw[5,:] = [-numpy.pi, numpy.pi] # allow any rotation around z - which is the axis normal to the tray top

    
    manip_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    tsr_chain = prpy.tsr.TSRChain(sample_start = False, sample_goal = True, constrain=False, 
                               TSR = manip_tsr)
    return [tsr_chain]


