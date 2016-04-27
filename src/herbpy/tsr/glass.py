import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import TSR, TSRChain

@TSRFactory('herb', 'plastic_glass', 'grasp')
def glass_grasp(robot, glass, manip=None, yaw_range=None, **kw_args):
    '''
    @param robot The robot performing the grasp
    @param glass The glass to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    @param yaw_range Allowable range of yaw around cup (default [-pi,pi])
    '''
    return _glass_grasp(robot, glass, manip=manip, yaw_range=yaw_range, **kw_args)
    
@TSRFactory('herb', 'plastic_glass', 'push_grasp')
def glass_push_grasp(robot, glass, manip=None, yaw_range=None, push_distance=0.1, **kw_args):
    '''
    This factory differes from glass_grasp in that it places the manipulator 
    further away and assumes the manip will perform a push after
    moving to this TSR.  This allows for dealing with uncertainty in pose estimation of the
    object. 
    After using this code to move the end-effector into place, the robot
    should push in the direction of the z-axis of the manipulator:
      direction = manip.GetEndEffectorTransform()[:3,2]

    @param robot The robot performing the grasp
    @param glass The glass to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    @param yaw_range Allowable range of yaw around cup (default [-pi,pi])
    @param push_distance The offset distance for pushing
    '''
    return _glass_grasp(robot, glass, manip=manip, yaw_range=yaw_range, push_distance=push_distance, **kw_args)

def _glass_grasp(robot, glass, manip=None, yaw_range=None, push_distance=0.0, **kw_args):
    """
    @param robot The robot performing the grasp
    @param glass The glass to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    @param yaw_range Allowable range of yaw around cup (default [-pi,pi])
    @param push_distance The offset distance for pushing
    """
    from prpy.tsr.generic import cylinder_grasp
    ee_to_palm = 0.18
    return cylinder_grasp(robot, glass, 
                          obj_radius = 0.045,
                          obj_height = 0.16,
                          lateral_offset = push_distance + ee_to_palm,
                          yaw_range = yaw_range,
                          manip = manip, **kw_args)
                
@TSRFactory('herb', 'plastic_glass', 'place')
def place_grasp(robot, glass, pose_tsr_chain, manip=None):
    '''
    Generates end-effector poses for placing the glass at a sampled pose
    This factory assumes the glass is grasped at the time it is called.
    
    @param robot The robot grasping the glass
    @param glass The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses for the glass
    @param manip The manipulator grasping the object, if None the active
       manipulator of the robot is used
    '''
    from prpy.tsr.generic import place_object
    return place_object(robot, glass, pose_tsr_chain, manip=manip)
                            
    
@TSRFactory('herb', 'plastic_glass', 'transport')
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
    from prpy.tsr.generic import transport_upright
    return transport_upright(robot, glass, manip=manip,
                             roll_epsilon=roll_epsilon,
                             pitch_epsilon=pitch_epsilon,
                             yaw_epsilon=yaw_epsilon)
