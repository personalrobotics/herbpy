import logging, openravepy, prpy
from prpy.action import ActionMethod
import numpy

logger = logging.getLogger('herbpy')

def kitchenFriendlyPlanner(robot):
    """
    Our normal planning stack includes TrajOpt. However, TrajOpt
    cannot deal with multiple robots in the environment. Since the
    fridge is treated as a robot (to have actuated joints) we re-create
    the planning stack from HERB not including TrajOpt
    @param robot The robot for planning
    """
    from prpy.planning import Sequence, FirstSupported
    from prpy.planning import NamedPlanner, TSRPlanner

    actual_planner = Sequence(robot.snap_planner, robot.vectorfield_planner)
    planner = FirstSupported(Sequence(actual_planner,
                            TSRPlanner(delegate_planner=actual_planner),
                            robot.cbirrt_planner),
                            NamedPlanner(delegate_planner=actual_planner))
    return planner

@ActionMethod
def DriveTo(robot, appliance, planning=True):
    """
    @param robot The robot driving
    @param appliance The appliance to drive to
    @param planning True if plan, False to teleport
    """
    appliance_pose = appliance.GetTransform() 
    robot_pose = numpy.eye(4)

    if appliance.GetName() == 'refrigerator':
        offset = numpy.array([1.4, -0.7, 0.0]) # jeking magic
    elif appliance.GetName() == 'dishwasher':
        offset = numpy.array([1.3, -0.7, 0.0])
    else:
        raise NameError("Appliance Name not recognized.")

    robot_pose[0:3, 3] = appliance_pose[0:3, 3] - offset
    if planning:
        robot.base.PlanToBasePose(robot_pose, execute=True)
    else:
        robot.SetTransform(robot_pose)

@ActionMethod
def GraspFridge(robot, fridge):
    """
    Action for grasping the door handle
    @param robot The robot to grasp
    @param fridge The kinbody representing the fridge
    """
    planner = kitchenFriendlyPlanner(robot)
    manip = robot.GetActiveManipulator()

    home_path = planner.PlanToNamedConfiguration(robot, 'home')
    robot.ExecutePath(home_path)

    # Create the grasp pose
    fridge_pose = fridge.GetTransform()

    # Get the lower handle pose
    lowerHandle = fridge.GetLink('lower_handle')
    lowerHandlePose = lowerHandle.GetTransform()

    # Now we need to find a grasp pose.
    # Translate the grasp pose to the left of the handle
    aabb = lowerHandle.ComputeAABB()
    graspPose = lowerHandlePose
    translationOffset = [-0.40, 0.1, 0]
    graspPose[0:3, 3] += translationOffset + (aabb.pos() - fridge_pose[0:3, 3])

    # Rotate the pose so that it aligns with the correct hand pose
    rot = openravepy.matrixFromAxisAngle([1, 0, 0], numpy.pi * 0.5)
    rot = rot.dot(openravepy.matrixFromAxisAngle([0, 1, 0], -numpy.pi * 0.5))
    graspPose = graspPose.dot(rot)
    last_rot = openravepy.matrixFromAxisAngle([0, 0, 1], numpy.pi)
    graspPose = graspPose.dot(last_rot)

    slow_velocity_limits = numpy.array([0.17, 0.17, 0.475, 0.475, 0.625, 0.625, 0.625])
    manip.SetVelocityLimits(2.0*slow_velocity_limits, min_accel_time=0.2)
    manip.hand.MoveHand(0.65, 0.65, 0.65, 0)

    pose_path = planner.PlanToEndEffectorPose(robot, graspPose)
    robot.ExecutePath(pose_path)

    manip.SetVelocityLimits(slow_velocity_limits, min_accel_time=0.2)
    # Move forward to touch the fridge
    manip.MoveUntilTouch([1, 0, 0], 0.1, ignore_collisions=[fridge])

    with prpy.rave.Disabled(fridge):
        # Move back
        manip.PlanToEndEffectorOffset([-1, 0, 0], 0.01, execute=True)

    # Move right
    manip.MoveUntilTouch([0, -1, 0], 0.05, ignore_collisions=[fridge])

    with prpy.rave.Disabled(fridge):
        # Center around the fridge
        manip.PlanToEndEffectorOffset([0, 1, 0], 0.01, execute=True)
        # Move back again
        manip.PlanToEndEffectorOffset([1, 0, 0], 0.045, execute=True)

    manip.hand.MoveHand(1.5, 1.5, 1.5)
    robot.Grab(fridge)
    manip.SetVelocityLimits(2.0*slow_velocity_limits, min_accel_time=0.2)

@ActionMethod
def OpenHandle(robot, fridge, manip=None, minopen=0, maxopen=None, render=True):
    """
    Action for opening the fridge
    @param robot The robot to grasp
    @param fridge The kinbody representing the fridge
    @param manip The manipulator that will open the door
    @param minopen The min amount to open the door
    @param maxopen The max amount to open the door
    @param render Whether to render the TSR
    """
    if manip is None:
        manip = robot.GetActiveManipulator()

    if maxopen is None:
        maxopen = minopen

    with robot.GetEnv():
        open_tsr = robot.tsrlibrary(fridge, 'open', manip, maxopen, minopen)

    goal_tsr = open_tsr[0]
    robot_goal = robot.right_arm.FindIKSolution(goal_tsr.sample(), openravepy.IkFilterOptions.CheckEnvCollisions)

    #FIXME need to extract proper fridge goal from sample
    fridge_goal = [0.1]
    robot_goal = numpy.append(robot_goal, fridge_goal)

    planner = fridgeFriendlyPlanner(robot)

    p = openravepy.KinBody.SaveParameters
    with robot.CreateRobotStateSaver(p.ActiveManipulator | p.ActiveDOF):
        robot.SetActiveManipulator(manip)
        robot.SetActiveDOFs(manip.GetArmIndices())
        with prpy.viz.RenderTSRList(open_tsr, robot.GetEnv(), render=render):
            # Release the fridge only for planning
            fridge.Enable(False)
            robot.Release(fridge)
            # Force to use cbirrt to get fridge plan as well
            path = robot.cbirrt_planner.PlanToTSR(robot, [open_tsr[1]], jointgoals=[robot_goal])
            traj = robot.PostProcessPath(path)
            fridge.Enable(True)
    robot.ExecuteTrajectory(traj)

    #FIXME doesnt actually move fridge door
    door_controller = openravepy.RaveCreateController(robot.GetEnv(), 'IdealController')
    fridge.SetController(door_controller)
    fridge.GetController().SetPath(traj)

    robot.Grab(fridge)
    return (path, traj)

@ActionMethod
def GraspDishwasher(robot, dishwasher):
    """
    Action for grasping the door handle
    @param robot The robot to grasp
    @param dishwasher The kinbody representing the dishwasher
    """
    planner = kitchenFriendlyPlanner(robot)
    manip = robot.GetActiveManipulator()

    home_path = planner.PlanToNamedConfiguration(robot, 'home')
    robot.ExecutePath(home_path)

    # Create the grasp pose
    dishwasher_pose = dishwasher.GetTransform()

    # Get the handle pose
    handle = dishwasher.GetLink('dish_handle')
    handlePose = handle.GetTransform()

    # Now we need to find a grasp pose.
    # Translate the grasp pose to the left of the handle
    graspPose = handlePose
    translationOffset = [-0.2, -0.2, 0.02]
    graspPose[0:3, 3] += translationOffset

    rot1 = openravepy.matrixFromAxisAngle([0, 1, 0], -numpy.pi*0.5)
    graspPose = graspPose.dot(rot1)
    rot2 = openravepy.matrixFromAxisAngle([0, 0, 1], numpy.pi)
    graspPose = graspPose.dot(rot2)

    slow_velocity_limits = numpy.array([0.17, 0.17, 0.475, 0.475, 0.625, 0.625, 0.625])
    manip.SetVelocityLimits(2.0*slow_velocity_limits, min_accel_time=0.2)
    manip.hand.MoveHand(0.65, 0.65, 0.65, 0)
    #return (graspPose, planner)

    # FIXME can never find goal IK ?
    pose_path = planner.PlanToEndEffectorPose(robot, graspPose)
    robot.ExecutePath(pose_path)

    """
    Untested, probably have to change directions
    manip.SetVelocityLimits(slow_velocity_limits, min_accel_time=0.2)
    # Move forward to touch the fridge
    manip.MoveUntilTouch([1, 0, 0], 0.1, ignore_collisions=[fridge])

    with prpy.rave.Disabled(fridge):
        # Move back
        manip.PlanToEndEffectorOffset([-1, 0, 0], 0.01, execute=True)

    # Move right
    manip.MoveUntilTouch([0, -1, 0], 0.05, ignore_collisions=[fridge])

    with prpy.rave.Disabled(fridge):
        # Center around the fridge
        manip.PlanToEndEffectorOffset([0, 1, 0], 0.01, execute=True)
        # Move back again
        manip.PlanToEndEffectorOffset([1, 0, 0], 0.045, execute=True)

    manip.hand.MoveHand(1.5, 1.5, 1.5)
    robot.Grab(fridge)
    manip.SetVelocityLimits(2.0*slow_velocity_limits, min_accel_time=0.2)
    """
