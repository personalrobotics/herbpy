import openravepy, os, time, prpy, actions, math
from prpy.action import ActionMethod
from herbpy.tsr.cup import *
from herbpy.tsr.pitcher import *

def PourWater(robot, cup, pitcher):
	"""
	@param robot The robot performing the pouring
	@param cup The cup to pour into
	@param pitcher The pitcher to pour
	"""	

	manip = robot.left_arm
    manip_cup = robot.right_arm

	# Grasp the handle
    success = actions.GraspHandle(robot, pitcher)

    #Lift Pitcher
    if success:
        success = actions.MoveObject(robot, direction=[0, 0, 1], distance=0.06)
    
    manip_pose = robot.GetActiveManipulator().GetEndEffectorTransform()   
    
    manip_cup.SetActive()
    robot.SetActiveManipulator(manip_cup)
    
    #Grasp cup
    success_cup = actions.PushGraspCup(robot, cup)

    pitcher_start = pitcher.GetTransform()
    pitcher_aabb = pitcher.ComputeAABB()

    if success_cup:
        success_cup = actions.MoveObject(robot, direction=[0, 0, 1], distance=0.01)

    manip.SetActive()
    robot.SetActiveManipulator(manip)

    if success_cup:
        p_tsr, min_tilt, max_tilt = robot.tsrlibrary(pitcher, 'pour')
        traj = manip.PlanToTSR(p_tsr, execute=False)
        cspec = traj.GetConfigurationSpecification()
        wpt = traj.GetWaypoint(traj.GetNumWaypoints() - 1)
        config = cspec.ExtractJointValues(wpt, robot, manip.GetArmIndices())
        with env, robot:
            manip.SetDOFValues(config)
            desired_cup_in_world_x = pitcher_start[0,3] + (2*pitcher_aabb.extents()[0])
            desired_cup_in_world_y = pitcher_start[1,3] - (2*pitcher_aabb.extents()[1])
            cup_in_world_x = cup_pose[0, 3]
            cup_in_world_y = cup_pose[1, 3]
            move_x = desired_cup_in_world_x - cup_in_world_x
            move_y = desired_cup_in_world_y - cup_in_world_y
            move_cup = math.sqrt(math.pow(move_x, 2) + math.pow(move_y, 2))
        manip_cup.SetActive()
        robot.SetActiveManipulator(manip_cup)
        success_cup = actions.MoveObject(robot, direction=[-1, -1, 0], distance=move_cup)

    if success_cup:
        success_cup = actions.MoveObject(robot, direction=[1, 0, 0], distance=cup_aabb.extents()[0]/2)

    if success_cup:
        success_cup = actions.MoveObject(robot, direction=[0, -1, 0], distance=cup_aabb.extents()[1])

    if success_cup:
        success_cup = actions.MoveObject(robot, direction=[0, 0, -1], distance=0.009)       

    manip.SetActive()
    robot.SetActiveManipulator(manip)

    # Pour
    # slow the arm down
    v = numpy.array([0.75, 0.75, 2., 2., 2.5, 2.5, 2.5])
    robot.GetActiveManipulator().SetVelocityLimits(0.5*v, 0.3)
    
    #Gets position of the pitcher and pours
    if success and success_cup:
        traj = robot.PostProcessPath(traj)
        robot.ExecuteTrajectory(traj)

    time.sleep(2)
    
    #Slows down the speed of the arm
    robot.GetActiveManipulator().SetVelocityLimits(v, 0.3)

    # Tilts pitcher back to original position
    if success:
        robot.ExecuteTrajectory(openravepy.planningutils.ReverseTrajectory(traj))
    
    if success:
        success = actions.MoveObject(robot, direction=-1.*manip_pose[:3,0], distance=0.1)
    
    pitcher_aabb = pitcher.ComputeAABB()
    pitcher_half = pitcher_aabb.extents()[2]
    pos = pitcher_aabb.pos()[2]
    move_down = pos - table_height - pitcher_half

    if success:
        success = actions.MoveObject(robot, direction=[0,0,-1], distance=move_down-0.05)
    
    #Open the hand and release the pitcher
    if success:
        manip.hand.OpenHand()
        robot.Release(pitcher)