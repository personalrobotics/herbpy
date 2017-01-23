from prpy.tsr.tsrlibrary import TSRFactory

@TSRFactory('herb', 'pop_tarts', 'grasp')
def poptarts_grasp(robot, pop_tarts, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop tarts box to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _poptarts_grasp(robot, pop_tarts, manip=manip)

@TSRFactory('herb', 'pop_tarts', 'push_grasp')
def poptarts_grasp(robot, pop_tarts, push_distance=0.1, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop_tarts to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _poptarts_grasp(robot, pop_tarts, push_distance=push_distance,
                           manip=manip)

def _poptarts_grasp(robot, pop_tarts, push_distance=0.0, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param pop_tarts The pop tarts box to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    from prpy.tsr.generic import box_grasp
    ee_to_palm_distance = 0.18
    return box_grasp(robot, pop_tarts,
                     length=0.08,
                     width=0.08,
                     height=0.16,
                     lateral_offset=ee_to_palm_distance + push_distance,
                     manip=manip, **kw_args)
