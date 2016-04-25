import logging, numpy, openravepy, prpy
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
from prpy.util import ComputeEnabledAABB
from block_description_generator.spatial_description_generator import spatial_desc_gen

logger = logging.getLogger('herbpy')

class NoTSRException(Exception):
    pass

def _GrabBlock(robot, blocks, table, manip=None, preshape=None,
              **kw_args):
    """
    @param robot The robot performing the grasp
    @param block The block to grab
    @param table The table, or object, the block is resting on
    @param manip The manipulator to use to grab the block
      (if None active manipulator is used)
    @param preshape The shape (dof_values) to move the hand to before
      executing the grab
    """
    from prpy.rave import AllDisabled, Disabled
    from prpy.viz import RenderTSRList, RenderVector

    env = robot.GetEnv()
    block = None

    if manip is None:
        with env:
            manip = robot.GetActiveManipulator()
    
    if preshape is None:
        preshape=[1.7, 1.7, 0.2, 2.45]

    # First move the hand to the right preshape
    manip.hand.MoveHand(*preshape)

    block_tsr_list = []
    with env:
        for b in blocks:
            # Get a TSR to move near the block.
            tsr_list = robot.tsrlibrary(b, 'grasp', manip=manip)
            block_tsr_list += tsr_list
    
    # Plan to a pose above the block
    with RenderTSRList(block_tsr_list, robot.GetEnv()):
        with Disabled(table, padding_only=True):
            manip.PlanToTSR(block_tsr_list, execute=True)

    with manip.GetRobot().GetEnv():
        ee_pose = manip.GetEndEffectorTransform()
    
    block_idxs = [ idx for idx, tsr_chain in enumerate(block_tsr_list)
                  if tsr_chain.contains(ee_pose) ]
    if len(block_idxs) == 0:
        raise NoTSRException("Failed to find the TSR PlanToTSR planned to")
    block = blocks[block_idxs[0]]
    
    # ---------------------------------------------------------
    # Shen Li
    # 1. AABB - bounding box
    #   AABB includes the information about transformation matrix (x,y of the CoM)
    #   (bb.GetConfigurationValues() or bb.GetTransform())
    # 2. name
    # 3. color
    # import IPython;
    # IPython.embed()


    import yaml
    import block_sorting.convert_color as closest_color
    block_data = {}
    for bb in blocks:
        block_name = bb.GetName() # string: b1, b2, ...
        block_data[block_name] = {}
        block_data[block_name]['name'] = str(block_name)

        # Get the color of the block
        geom = bb.GetLinks()[0].GetGeometries()[0]
        color = geom.GetDiffuseColor()
        # Re-scale the block's RGB color from float to 0-255
        block_color = (color[0]*255.0, color[1]*255.0, color[2]*255.0)
        # Get the English color name that most closely matches the RGB value
        actual_color_name, closest_color_name = closest_color.get_colour_name(block_color)
        block_data[block_name]['color_RGB'] = block_color
        block_data[block_name]['color_name'] = str(closest_color_name)

        bounding_box = bb.ComputeAABB()

        # we need to reverse x with y because now the y is actually horizontal
        # for example, in this example, the robot is closest to the yellow one,
        # so we have to rotate the table to make it consistent
        block_data[block_name]['AABB_pos'] = [bounding_box.pos()[1], bounding_box.pos()[0]]
        block_data[block_name]['AABB_ext'] = [bounding_box.extents()[1], bounding_box.extents()[0]]

    table_AABB_pos = [table.ComputeAABB().pos()[1], table.ComputeAABB().pos()[0]]
    table_AABB_ext = [table.ComputeAABB().extents()[1], table.ComputeAABB().extents()[0]]
    # for key, value in block_data.iteritems():
        # print value
    # with open('./block_scenario.yml', 'w') as outfile:
        # outfile.write( yaml.dump(block_data, default_flow_style=False) )
    target_block_name = block.GetName()
    num_of_solu_needed = 1
    print 'spatial_desc_gen starts-----------------------------'

    desc_list = spatial_desc_gen(block_data, str(target_block_name),\
        table_AABB_pos, table_AABB_ext, False, num_of_solu_needed)
    if not desc_list:
        desc_list = []
        desc_list[0] = 'Sorry, I could not describe this block I am going to pick up.'
    else:
        print 'demo.py: Spatial description of this block:'
        for i in xrange(len(desc_list)):
            print 'Solution '+str(i+1)+' : '+str(desc_list[i])

        best_solution = desc_list[0]
        print 'best_solution='+ best_solution
        print 'spatial_desc_gen ends-----------------------------'

    import IPython;IPython.embed()
    robot.Say(desc_list[0])
    import IPython;IPython.embed()

    # ---------------------------------------------------------

    
    try:
        with AllDisabled(env, [table] + blocks, padding_only=True):
            # Move down until touching the table
            with env:
                start_point = manip.GetEndEffectorTransform()[0:3, 3]
                table_aabb = ComputeEnabledAABB(table)
                table_height = table_aabb.pos()[2] + table_aabb.extents()[2]
                # 0.14 is the distance from finger tip to end-effector frame
                current_finger_height = manip.GetEndEffectorTransform()[2,3] - 0.14
                min_distance = 0.10# current_finger_height - table_height

            down_direction = [0., 0., -1.]
            with RenderVector(start_point, down_direction, min_distance, env):
                manip.MoveUntilTouch(direction=down_direction, timelimit=5,
                    distance=min_distance, max_distance=min_distance + 0.05,
                    ignore_collisions=blocks + [table])

            # Move parallel to the table to funnel the block into the fingers
            # by projecting the -x direction of end-effector onto the xy-plane.
            with env:
                start_point = manip.GetEndEffectorTransform()[0:3, 3]
                funnel_direction = -1. * manip.GetEndEffectorTransform()[:3,0]
                funnel_direction[2] = 0.

            # TODO: We should only have to disable the block for this. Why does
            # this fail if we do not disable the table?
            with AllDisabled(env, blocks + [table]):
                with RenderVector(start_point, funnel_direction, 0.1, env):
                    manip.PlanToEndEffectorOffset(direction=funnel_direction,
                                                  distance=0.08, max_distance=0.12, 
                                                  timelimit=5., execute=True)
        
        # Close the finger to grab the block
        manip.hand.MoveHand(f3=1.7)
            
        # Compute the pose of the block in the hand frame
        with env:
            local_p = [0.01, 0, 0.24, 1.0]
            hand_pose = manip.GetEndEffectorTransform()
            world_p = numpy.dot(hand_pose, local_p)
            block_pose = block.GetTransform()
            block_pose[:,3] = world_p
            block_relative = numpy.dot(numpy.linalg.inv(hand_pose), block_pose)
        
        # Now lift the block up off the table
        with AllDisabled(env, blocks + [table]):
            manip.PlanToEndEffectorOffset(direction=[0, 0, 1], distance=0.05, 
                                          timelimit=5, execute=True)
            
        # OpenRAVE trick to hallucinate the block into the correct pose relative to the hand
        with env:
            hand_pose = manip.GetEndEffectorTransform()
            block_pose = numpy.dot(hand_pose, block_relative)
            block.SetTransform(block_pose)
            manip.GetRobot().Grab(block)
        return block
    except PlanningError as e:
        logger.error('Failed to complete block grasp')
        raise

@ActionMethod
def GrabBlocks(robot, blocks, table, **kw_args):
    """
    @param robot The robot performing the grasp
    @param block The block to grab
    @param table The table, or object, the block is resting on
    @param manip The manipulator to use to grab the block
      (if None active manipulator is used)
    """
    return _GrabBlock(robot, blocks, table, **kw_args)

@ActionMethod
def GrabBlock(robot, block, table, **kw_args):
    """
    @param robot The robot performing the grasp
    @param block The block to grab
    @param table The table, or object, the block is resting on
    @param manip The manipulator to use to grab the block
      (if None active manipulator is used)
    """
    return _GrabBlock(robot, [block], table, **kw_args)

@ActionMethod
def PlaceBlock(robot, block, on_obj, manip=None, **kw_args):
    """
    Place a block on an object. This function assumes the block
    is grabbed by the robot when the method is called.
    @param robot The robot performing the block placement
    @param block The block to be placed
    @param on_obj The object to place the block on, this object
    must have a 'point_tsr defined
    @param manip The manipulator used to perform the placement
    If none, the active manipulator is used.
    """
    env = robot.GetEnv()

    # Get a tsr for this position
    with env:
        object_place_list = robot.tsrlibrary(on_obj, 'point_on', manip=manip)
        place_tsr_list = robot.tsrlibrary(block, 'place_on', 
                                          pose_tsr_chain=object_place_list[0], 
                                          manip=manip)

    # Plan there
    with prpy.viz.RenderTSRList(object_place_list, robot.GetEnv()):
        manip.PlanToTSR(place_tsr_list, execute=True)

    # Open the hand and drop the block
    manip.hand.MoveHand(f3=0.2)

    with env:
        manip.GetRobot().Release(block)

        # Move the block down until it hits something
        block_pose = block.GetTransform()
        while not env.CheckCollision(block) and block_pose[2,3] > 0.0:
            block_pose[2,3] -= 0.02
            block.SetTransform(block_pose)
