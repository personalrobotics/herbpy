import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain

@TSRFactory('herb', 'pop_tarts', 'grasp')
def poptarts_grasp(robot, pop_tarts, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop tarts box to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _poptarts_grasp(robot, pop_tarts, manip = manip)

@TSRFactory('herb', 'pop_tarts', 'push_grasp')
def poptarts_grasp(robot, pop_tarts, push_distance = 0.1, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop_tarts to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _poptarts_grasp(robot, pop_tarts, push_distance = push_distance, manip = manip)

def _poptarts_grasp(robot, pop_tarts, push_distance = 0.0, manip = None, **kw_args):
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
    
    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, 
            constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
