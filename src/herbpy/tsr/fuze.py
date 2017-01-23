from prpy.tsr.tsrlibrary import TSRFactory

@TSRFactory('herb', 'fuze_bottle', 'grasp')
def fuze_grasp(robot, fuze, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _fuze_grasp(robot, fuze, manip=manip)

@TSRFactory('herb', 'fuze_bottle', 'push_grasp')
def fuze_grasp(robot, fuze, push_distance=0.1, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _fuze_grasp(robot, fuze, push_distance=push_distance, manip=manip)

def _fuze_grasp(robot, fuze, push_distance=0.0, manip=None, **kw_args):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    from prpy.tsr.generic import cylinder_grasp
    ee_to_palm_distance = 0.18
    return cylinder_grasp(robot, fuze, obj_radius=0.05,
                          obj_height=0.216,
                          lateral_offset=ee_to_palm_distance + push_distance,
                          manip=manip)
                
@TSRFactory('herb', 'fuze_bottle', 'place')
def fuze_on_table(robot, fuze, pose_tsr_chain, manip=None):
    """
    Generates end-effector poses for placing the fuze
    This factory assumes the fuze is grasped at the time it is called.
    
    @param robot The robot grasping the fuze
    @param fuze The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses
                          for the fuze
    @param manip The manipulator grasping the object, if None the active
       manipulator of the robot is used
    """
    from prpy.tsr.generic import place_object
    return place_object(robot, fuze, pose_tsr_chain, manip=manip)
                            
    
@TSRFactory('herb', 'fuze_bottle', 'transport')
def fuze_transport(robot, fuze, manip=None, roll_epsilon=0.2, pitch_epsilon=0.2, yaw_epsilon=0.2):
    """
    Generates a trajectory-wide constraint for transporting the object with
    little roll, pitch or yaw. Assumes the object has already been grasped
    and is in the proper configuration for transport.

    @param robot The robot grasping the fuze
    @param fuze The grasped object
    @param manip the manipulator grasping the object, if None the active
                  manipulator of the robot is used
    @param roll_epsilon The amount to let the object roll during
                        transport (object frame)
    @param pitch_epsilon The amount to let the object pitch during
                         transport (object frame)
    @param yaw_epsilon The amount to let the object yaw during
                         transport (object frame)
    """
    from prpy.tsr.generic import transport_upright_tsr
    return transport_upright_tsr(robot, fuze, manip=manip,
                                 roll_epsilon=roll_epsilon,
                                 pitch_epsilon=pitch_epsilon,
                                 yaw_epsilon=yaw_epsilon)
