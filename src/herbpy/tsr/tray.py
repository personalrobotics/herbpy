import numpy, prpy
import prpy.tsr

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'wicker_tray', 'point_on')
def point_on(robot, tray, manip=None, padding=0.0, handle_padding=True):
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
            
    T0_w = tray.GetTransform()

    # The frame is set on the ta such that the y-axis is normal to the table surface
    Tw_e = numpy.eye(4)
    Tw_e[2,3] = 0.04 # set the object on top of the tray

    Bw = numpy.zeros((6,2))

    xdim = max(0.205 - padding, 0.0)
    if handle_padding:
        ydim = max(0.25 - padding, 0.0)
    else:
        ydim = max(0.3 - padding, 0.0)
    Bw[0,:] = [-xdim, xdim ] # move along x and y directions to get any point on tray
    Bw[1,:] = [-ydim, ydim]
    Bw[2,:] = [-0.02, 0.04] # verticle movement
    Bw[5,:] = [-numpy.pi, numpy.pi-.0001] # allow any rotation around z - which is the axis normal to the tray top

    
    tray_top_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    tray_top_chain = prpy.tsr.TSRChain(sample_start = False, sample_goal = True, constrain=False, 
                               TSR = tray_top_tsr)
    return [tray_top_chain]


@prpy.tsr.tsrlibrary.TSRFactory('herb', 'wicker_tray', 'handle_grasp')
def handle_grasp(robot, tray, manip=None, handle=None):
    '''
    This creates a TSR for grasping the left handle of the tray
    By default, the handle is grasped with the left hand, unless manip is specified

    @param robot The robot performing the grasp
    @param tray The tray to grasp
    @param manip The manipulator to perform the grasp, if None
      the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()
            
    tray_in_world = tray.GetTransform()

    # Compute the pose of both handles in the tray
    handle_one_in_tray = numpy.eye(4)
    handle_one_in_tray[1,3] = -0.33
    
    handle_two_in_tray = numpy.eye(4)
    handle_two_in_tray[1,3] = 0.33

    handle_poses = [handle_one_in_tray, handle_two_in_tray]

    # Define the grasp relative to a particular handle
    grasp_in_handle = numpy.array([[0.,  1.,  0., 0.],
                                   [1.,  0.,  0., 0.],
                                   [0.,  0., -1., 0.33],
                                   [0.,  0.,  0., 1.]])

    Bw = numpy.zeros((6,2))
    epsilon = 0.05
    Bw[0,:] = [0., epsilon] # Move laterally along handle
    Bw[2,:] = [-0.01, 0.01] # Move up or down a little bit
    Bw[5,:] = [-5.*numpy.pi/180., 5.*numpy.pi/180.] # Allow 5 degrees of yaw

    # Now build tsrs for both
    chains = []
    best_dist = float('inf')
    for handle_in_tray in handle_poses:
        handle_in_world = numpy.dot(tray_in_world, handle_in_tray)
        dist = numpy.linalg.norm(handle_in_world[:2,3] - manip.GetEndEffectorTransform()[:2,3])
        print 'Dist: ', dist, 'best dist: ', best_dist
        if handle == 'closest' and dist > best_dist:
            continue
        tray_grasp_tsr = prpy.tsr.TSR(T0_w = handle_in_world, 
                                      Tw_e = grasp_in_handle, 
                                      Bw = Bw, 
                                      manip=manip_idx)
        tray_grasp_chain = prpy.tsr.TSRChain(sample_start = False, 
                                             sample_goal = True, 
                                             constrain=False,
                                             TSR = tray_grasp_tsr)
        if handle == 'closest':
            chains = []
        chains.append(tray_grasp_chain)
        best_dist = dist
    return chains

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'wicker_tray', 'lift')
def lift(robot, tray, distance=0.1):
    '''
    This creates a TSR for lifting the tray a specified distance with both arms
    It is assumed that when called, the robot is grasping the tray with both arms

    @param robot The robot to perform the lift
    @param tray The tray to lift
    @param distance The distance to lift the tray
    '''
    print 'distance = %0.2f' % distance

    with robot:
        robot.left_arm.SetActive()
        left_manip_idx = robot.GetActiveManipulatorIndex()

        robot.right_arm.SetActive()
        right_manip_idx = robot.GetActiveManipulatorIndex()

    # First TSR defines a goal for the left arm
    left_in_world = robot.left_arm.GetEndEffectorTransform()
    desired_left_in_world = robot.left_arm.GetEndEffectorTransform()
    desired_left_in_world[2,3] += distance

    Bw_left = numpy.zeros((6,2))
    epsilon = 0.05
    Bw_left[0,:] = [-epsilon, epsilon]
    Bw_left[1,:] = [-epsilon, epsilon]
    Bw_left[4,:] = [-epsilon, epsilon] # a little bit of tilt around handle

    tsr_left_goal = prpy.tsr.TSR(T0_w = desired_left_in_world,
                                 Tw_e = numpy.eye(4),
                                 Bw = Bw_left,
                                 manip = left_manip_idx)

    # Now define a TSR that is the right hand relative to the left
    right_in_world = robot.right_arm.GetEndEffectorTransform()
    right_in_left = numpy.dot(numpy.linalg.inv(left_in_world), right_in_world)

    Bw_right = numpy.zeros((6,2))
    tsr_right_goal = prpy.tsr.TSR(T0_w = numpy.eye(4), # overwritten in planner
                                  Tw_e = right_in_left,
                                  Bw = Bw_right,
                                  manip = right_manip_idx)
    goal_tsr_chain = prpy.tsr.TSRChain(sample_start = False,
                                       sample_goal = True,
                                       constrain = False,
                                       TSRs = [tsr_left_goal, tsr_right_goal])
    
    # Now define a TSR that constrains the movement of the arms
    Bw_constrain = numpy.zeros((6,2))
    Bw_constrain[0,:] = [-epsilon, epsilon]
    Bw_constrain[1,:] = [-epsilon, epsilon]
    if distance < 0:
        Bw_constrain[2,:] = [-epsilon+distance, epsilon]
    else:
        Bw_constrain[2,:] = [-epsilon, epsilon+distance]

    Bw_constrain[3,:] = [-epsilon, epsilon]
    Bw_constrain[4,:] = [-epsilon, epsilon]
    Bw_constrain[5,:] = [-epsilon, epsilon]

    tsr_left_constraint = prpy.tsr.TSR(T0_w = left_in_world,
                                       Tw_e = numpy.eye(4),
                                       Bw = Bw_constrain,
                                       manip = left_manip_idx)

    tsr_right_constraint = prpy.tsr.TSR(T0_w = numpy.eye(4),
                                        Tw_e = right_in_left,
                                        Bw = numpy.zeros((6,2)),#Bw_constrain,
                                        manip=right_manip_idx,
                                        bodyandlink='%s %s' % (robot.GetName(), robot.left_arm.GetEndEffector().GetName()))
    movement_chain = prpy.tsr.TSRChain(sample_start = False, sample_goal = False, constrain=True,
                              TSRs = [tsr_right_constraint, tsr_left_constraint])
    
    return [goal_tsr_chain, movement_chain] 

@prpy.tsr.tsrlibrary.TSRFactory('herb', 'wicker_tray', 'pull')
def pull_tray(robot, tray, manip=None, max_distance=0.0, min_distance=0.0, 
              direction=[1., 0., 0.], angular_tolerance=[0., 0., 0.],  position_tolerance=[0., 0., 0.]):
    """
    This creates a TSR for pulling the tray in a specified direction for a specified distance
    It is assumed that when called, the robot is grasping the tray

    @param robot The robot to perform the lift
    @param tray The tray to lift
    @param manip The manipulator to pull with (if None the active manipulator is used)
    @param max_distance The max distance to pull the tray
    @param min_distance The min distance to pull the tray
    @param angular_tolerance A 3x1 vector describing the tolerance of the pose of the end-effector
          in roll, pitch and yaw relative to a coordinate frame with z pointing in the pull direction
    @param position_tolerance A 3x1 vector describing the tolerance of the pose of the end-effector
          in x, y and z relative to a coordinate frame with z pointing in the pull direction
    """
    if manip is None:
        manip = robot.GetActiveManipulator()
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()
            
    # Create a w frame with z-axis pointing in direction of pull
    ee_in_world = manip.GetEndEffectorTransform()
    w_in_world = prpy.kin.H_from_op_diff(ee_in_world[:3,3], direction)

    # Compute the current end-effector in w frame
    ee_in_w = numpy.dot(numpy.linalg.inv(w_in_world), ee_in_world)

    # Convert the position and angular tolerances from end-effector frame
    #  to world frame
    position_tolerance = abs(numpy.dot(ee_in_w[:3,:3], position_tolerance))
    angular_tolerance = abs(numpy.dot(ee_in_w[:3,:3], angular_tolerance))

    # Move the w frame the appropriate distance along the pull direction
    end_in_w = numpy.eye(4)
    end_in_w[2,3] = max_distance
    desired_w_in_world = numpy.dot(w_in_world, end_in_w)
    
    distance_diff = max_distance - min_distance
    Bw_goal = numpy.zeros((6,2))
    Bw_goal[0,:] = [-position_tolerance[0], position_tolerance[0]]
    Bw_goal[1,:] = [-position_tolerance[1], position_tolerance[1]]
    Bw_goal[2,:] = [-(position_tolerance[2] + distance_diff), position_tolerance[2]]
    Bw_goal[3,:] = [-angular_tolerance[0], angular_tolerance[0]]
    Bw_goal[4,:] = [-angular_tolerance[1], angular_tolerance[1]]
    Bw_goal[5,:] = [-angular_tolerance[2], angular_tolerance[2]]
    
    goal_tsr = prpy.tsr.TSR(T0_w = desired_w_in_world,
                            Tw_e = ee_in_w,
                            Bw = Bw_goal,
                            manip = manip_idx)

    goal_tsr_chain = prpy.tsr.TSRChain(sample_start=False, 
                                       sample_goal=True, 
                                       constrain=False,
                                       TSRs=[goal_tsr])

    Bw_constraint = numpy.zeros((6,2))
    Bw_constraint[0,:] = [-position_tolerance[0], position_tolerance[0]]
    Bw_constraint[1,:] = [-position_tolerance[1], position_tolerance[1]]
    Bw_constraint[2,:] = [-position_tolerance[2], max_distance + position_tolerance[2]]
    Bw_constraint[3,:] = [-angular_tolerance[0], angular_tolerance[0]]
    Bw_constraint[4,:] = [-angular_tolerance[1], angular_tolerance[1]]
    Bw_constraint[5,:] = [-angular_tolerance[2], angular_tolerance[2]]

    traj_tsr = prpy.tsr.TSR(T0_w = w_in_world,
                            Tw_e = ee_in_w,
                            Bw = Bw_constraint,
                            manip = manip_idx)
    traj_tsr_chain = prpy.tsr.TSRChain(sample_start=False,
                                       sample_goal=False,
                                       constrain=True,
                                       TSRs = [traj_tsr])
    
    return [goal_tsr_chain, traj_tsr_chain]
