import openravepy, time, prpy, math, numpy
from prpy.action import ActionMethod

@ActionMethod
def MoveCupAndPour(robot, table, manip_pitcher, manip_cup, cup, pitcher):
    """
    @param robot The robot performing the pouring
    @param table The table the objects are on
    @param manip_pitcher The arm that is manipulating the pitcher
    @param manip_cup The arm that is manipulating the cup 
    @param cup The cup to pour into
    @param pitcher The pitcher to pour
    """	

    env = robot.GetEnv()
    with env:
        pitcher_start = pitcher.GetTransform()
        pitcher_aabb = pitcher.ComputeAABB()
        cup_pose = cup.GetTransform()
        cup_aabb = cup.ComputeAABB()
        manip_cup_pose = manip_cup.GetEndEffectorTransform()
        manip_pitcher_pose = manip_pitcher.GetEndEffectorTransform()

    p_tsr, min_tilt, max_tilt = robot.tsrlibrary(pitcher, 'pour')
    traj = manip_pitcher.PlanToTSR(p_tsr, execute=False)
    cspec = traj.GetConfigurationSpecification()
    wpt = traj.GetWaypoint(traj.GetNumWaypoints() - 1)
    config = cspec.ExtractJointValues(wpt, robot, manip_pitcher.GetArmIndices())
    with env:
        with robot.CreateRobotStateSaver(openravepy.KinBody.SaveParameters.LinkTransformation):
            manip_pitcher.SetDOFValues(config)
            desired_cup_in_world_x = pitcher_start[0,3] + (2*pitcher_aabb.extents()[0])
            desired_cup_in_world_y = pitcher_start[1,3] - (2*pitcher_aabb.extents()[1])
            cup_in_world_x = cup_pose[0, 3]
            cup_in_world_y = cup_pose[1, 3]
            move_x = desired_cup_in_world_x - cup_in_world_x + cup_aabb.extents()[0]/2
            move_y = desired_cup_in_world_y - cup_in_world_y - cup_aabb.extents()[1]
            move_cup = math.sqrt(move_x ** 2 + move_y ** 2)

    from prpy.rave import Disabled
    with Disabled(table):
        manip_cup.PlanToEndEffectorOffset(direction=[move_x, move_y, 0], distance=move_cup, position_tolerance=0.1, execute=True, timelimit=10)
        manip_cup.PlanToEndEffectorOffset(direction=[0, 0, -1], distance=0.007, position_tolerance=0.1, execute=True, timelimit=10)
        manip_cup.hand.OpenHand()
        with env:
            robot.Release(cup)
        manip_cup.PlanToEndEffectorOffset(direction=-1*manip_cup_pose[:3,2], distance=0.1, position_tolerance=0.1, execute=True, timelimit=10)
        manip_cup.PlanToNamedConfiguration('home', execute=True)

    # Pour
    # slow the arm down
    v = numpy.array([0.75, 0.75, 2., 2., 2.5, 2.5, 2.5])
    with robot.CreateRobotStateSaver(openravepy.KinBody.SaveParameters.JointMaxVelocityAndAcceleration):
        manip_pitcher.SetVelocityLimits(0.5*v, 0.3)
        
        #Gets position of the pitcher and pours
        traj = robot.PostProcessPath(traj)
        robot.ExecuteTrajectory(traj)
        time.sleep(1) #To let the water pour from pitcher into cup

    # Tilts pitcher back to original position
    robot.ExecuteTrajectory(openravepy.planningutils.ReverseTrajectory(traj))
    
    manip_pitcher.PlanToEndEffectorOffset(direction=-1*manip_pitcher_pose[:3,0], distance=0.1, position_tolerance=0.1, execute=True)
