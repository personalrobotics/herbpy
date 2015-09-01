import logging, numpy, openravepy, prpy
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError

logger = logging.getLogger('herbpy')

@ActionMethod
def GrabBlock(robot, block, table, manip=None, preshape=[1.7, 1.7, 0.2, 2.45],
              **kw_args):
    """
    @param robot The robot performing the grasp
    @param block The block to grab
    @param table The table, or object, the block is resting on
    @param manip The manipulator to use to grab the block
      (if None active manipulator is used)
    """
    if manip is None:
        manip = robot.GetActiveManipulator()
    
    # First move the hand to the right preshape
    manip.hand.MoveHand(*preshape)

    offset = 0.05 #vertical offset relative to block
    alpha = 0.5 # orientation of end-effector relative to end-effector

    block_in_world = block.GetTransform()
    ee_in_block = numpy.array([[numpy.cos(alpha), 0., -numpy.sin(alpha), 0.3*numpy.sin(alpha)+0.04],
                               [ 0., -1, 0, 0.],
                               [-numpy.sin(alpha), 0., -numpy.cos(alpha), 0.25+offset],
                               [ 0., 0., 0., 1.]])
        
    ee_in_world = numpy.dot(block_in_world, ee_in_block)
    
    # Plan to a pose above the block
    manip.PlanToEndEffectorPose(ee_in_world)

    # Move down until touching the table
    try:
        #self.manip.MoveUntilTouch(direction=[0, 0, -1], distance=offset*2.)
        table_aabb = table.ComputeAABB()
        table_height = table_aabb.pos()[2] + table_aabb.extents()[2] + 0.01
        desired_ee_height = 0.204 + table_height
        current_ee_height = manip.GetEndEffectorTransform()[2,3]
        with prpy.rave.Disabled(table):
            with prpy.rave.Disabled(block):
                manip.PlanToEndEffectorOffset(direction=[0, 0, -1], distance=current_ee_height - desired_ee_height, timelimit=15)

                # Now move the hand parallel to the table to funnel the block into the fingers
                funnel_direction = [0,1,0];
                manip.PlanToEndEffectorOffset(direction=funnel_direction, distance=0.04, max_distance=0.1)
        
        # Close the finger to grab the block
        manip.hand.MoveHand(f3=1.7)
            
        # Compute the pose of the block in the hand frame
        local_p = [0.01, 0, 0.24, 1.0]
        hand_pose = manip.GetEndEffectorTransform()
        world_p = numpy.dot(hand_pose, local_p)
        block_pose = block.GetTransform()
        block_pose[:,3] = world_p
        block_relative = numpy.dot(numpy.linalg.inv(hand_pose), block_pose)
        
        # Now lift the block up off the table
        with prpy.rave.Disabled(table):
            with prpy.rave.Disabled(block):
                manip.PlanToEndEffectorOffset(direction=[0, 0, 1], distance=0.2)
            
        # OpenRAVE trick to hallucinate the block into the correct pose relative to the hand
        hand_pose = manip.GetEndEffectorTransform()
        block_pose = numpy.dot(hand_pose, block_relative)
        block.SetTransform(block_pose)
        manip.GetRobot().Grab(block)

    except PlanningError, e:
        logger.error('Failed to complete block grasp')
        raise

@ActionMethod
def PlaceBlock(robot, block, on_obj, center=True, **kw_args):
    pass
