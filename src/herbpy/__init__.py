import roslib; roslib.load_manifest('herbpy')
import openrave_exports; openrave_exports.export()
import logging, openravepy, or_multi_controller, types
import cbirrt, chomp
from planner import PlanningError 

NODE_NAME = 'herbpy'
OPENRAVE_FRAME_ID = '/openrave'
HEAD_DOFS = [ 22, 23 ]

def LookAt(robot, target, execute=True):
    # Find an IK solution to look at the point.
    ik_params = openravepy.IkParameterization(target, openravepy.IkParameterization.Type.Lookat3D)
    target_dof_values = robot.head.ik_database.manip.FindIKSolution(ik_params, 0)
    if target_dof_values == None:
        return None

    # Create a two waypoint trajectory for the head.
    current_dof_values = robot.GetDOFValues(robot.head.GetArmIndices())
    config_spec = robot.head.GetArmConfigurationSpecification()
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
    traj.Init(config_spec)
    traj.Insert(0, current_dof_values, config_spec)
    traj.Insert(1, target_dof_values, config_spec)

    # Optionally exeucute the trajectory.
    if execute:
        robot.head.arm_controller.SetPath(traj)
        # TODO: Implement a more efficient way of waiting for a single
        # controller to finish.
        while not robot.head.arm_controller.IsDone():
            pass

    return traj

def PlanGeneric(robot, command_name, execute=True, *args, **kw_args):
    # Sequentially try each planner until one succeeds.
    traj = None
    for planner in robot.planners:
        try:
            command = getattr(planner, command_name)
            print command
            traj = command(*args, **kw_args)
            break
        except NotImplementedError:
            pass
        except PlanningError, e:
            logging.warning('Planning with {0:s} failed: {1:s}'.format(planner.GetName(), e))

    if traj is None:
        logging.error('Planning failed with all planners.')
        return None

    # TODO: Retime the trajectory.
    # TODO: Optionally execute the trajectory.
    return traj

def PlanToConfiguration(robot, goal, **kw_args):
    return PlanGeneric(robot, 'PlanToConfiguration', goal, **kw_args)

def PlanToEndEffectorPose(robot, goal_pose, **kw_args):
    return PlanGeneric(robot, 'PlanToEndEffectorPose', goal_pose, **kw_args)

def SetStiffness(manipulator, stiffness):
    try:
        manipulator.arm_controller.SendCommand('SetStiffness {0:f}'.format(stiffness))
        return True
    except openravepy.openrave_exception, e:
        logging.error(e)
        return False

def attach_controller(robot, name, controller_args, dof_indices, affine_dofs, simulation):
    if simulation:
        controller_args = 'IdealController'

    delegate_controller = openravepy.RaveCreateController(robot.GetEnv(), controller_args)
    robot.multicontroller.attach(name, delegate_controller, dof_indices, affine_dofs)
    return delegate_controller

def initialize_manipulator(manipulator):
    manipulator.SetStiffness = types.MethodType(SetStiffness, manipulator, type(manipulator))

def initialize_controllers(robot, left_arm_sim, right_arm_sim, left_hand_sim, right_hand_sim,
                                  head_sim, segway_sim):
    head_args = 'OWDController {0:s} {1:s}'.format(NODE_NAME, '/head/owd')
    left_arm_args = 'OWDController {0:s} {1:s}'.format(NODE_NAME, '/left/owd')
    right_arm_args = 'OWDController {0:s} {1:s}'.format(NODE_NAME, '/right/owd')
    left_hand_args = 'BHController {0:s} {1:s}'.format(NODE_NAME, '/left/bhd')
    right_hand_args = 'BHController {0:s} {1:s}'.format(NODE_NAME, '/right/bhd')
    base_args = 'SegwayController {0:s}'.format(NODE_NAME)

    # Create aliases for the manipulators.
    left_arm_dofs = robot.left_arm.GetArmIndices()
    right_arm_dofs = robot.right_arm.GetArmIndices()
    left_hand_dofs = robot.left_arm.GetChildDOFIndices()
    right_hand_dofs = robot.right_arm.GetChildDOFIndices()

    # Controllers.
    robot.multicontroller = or_multi_controller.MultiControllerWrapper(robot)
    robot.head.arm_controller = attach_controller(robot, 'head', head_args, HEAD_DOFS, 0, head_sim)
    robot.left_arm.arm_controller = attach_controller(robot, 'left_arm', left_arm_args, left_arm_dofs, 0, left_arm_sim)
    robot.right_arm.arm_controller = attach_controller(robot, 'right_arm', right_arm_args, right_arm_dofs, 0, right_arm_sim)
    robot.left_arm.hand_controller = attach_controller(robot, 'left_hand', left_hand_args, left_hand_dofs, 0, left_hand_sim)
    robot.right_arm.hand_controller = attach_controller(robot, 'right_hand', right_hand_args, right_hand_dofs, 0, right_hand_sim)
    robot.segway_controller = attach_controller(robot, 'base', base_args, [], openravepy.DOFAffine.Transform, segway_sim)
    robot.controllers = [ robot.head.arm_controller, robot.segway_controller,
                          robot.left_arm.arm_controller, robot.right_arm.arm_controller,
                          robot.left_arm.hand_controller, robot.right_arm.hand_controller ]
    robot.multicontroller.finalize()

def initialize_sensors(robot, moped_sim=True):
    moped_args = 'MOPEDSensorSystem {0:s} {1:s} {2:s}'.format(NODE_NAME, '/moped', OPENRAVE_FRAME_ID)

    if not moped_sim:
        self.moped_sensorsystem = openravepy.RaveCreateSensorSystem(self.env, args)

def initialize_herb(robot, left_arm_sim=False, right_arm_sim=False,
                           left_hand_sim=False, right_hand_sim=False,
                           head_sim=False, segway_sim=False, moped_sim=False):
    robot.left_arm = robot.GetManipulator('left_wam')
    robot.right_arm = robot.GetManipulator('right_wam')
    robot.head = robot.GetManipulator('head_wam')

    # Initialize the OpenRAVE plugins.
    initialize_controllers(robot, left_arm_sim=left_arm_sim, right_arm_sim=right_arm_sim,
                                  left_hand_sim=left_hand_sim, right_hand_sim=right_hand_sim,
                                  head_sim=head_sim, segway_sim=segway_sim)
    initialize_sensors(robot, moped_sim=moped_sim)

    # Load the IK database for the head.
    with robot.GetEnv():
        robot.SetActiveManipulator('head_wam')
        robot.head.ik_database = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot, iktype=openravepy.IkParameterizationType.Lookat3D)
        if not robot.head.ik_database.load():
            logging.info('Generating IK database for the head.')
            robot.head.ik_database.autogenerate()

    # Wait for the robot's state to update.
    for controller in robot.controllers:
        try:
            controller.SendCommand('WaitForUpdate')
        except openravepy.openrave_exception, e:
            pass

    # Configure the planners.
    robot.cbirrt_planner = cbirrt.CBiRRTPlanner(robot)
    robot.chomp_planner = chomp.CHOMPPlanner(robot)
    robot.planners = [ robot.cbirrt_planner, robot.chomp_planner ]

    # Bind extra methods onto the OpenRAVE robot.
    robot.PlanToConfiguration = types.MethodType(PlanToConfiguration, robot, type(robot))
    robot.PlanToEndEffectorPose = types.MethodType(PlanToEndEffectorPose, robot, type(robot))
    robot.LookAt = types.MethodType(LookAt, robot, type(robot))

    # Bind extra methods to the manipulators.
    initialize_manipulator(robot.left_arm)
    initialize_manipulator(robot.right_arm)
    initialize_manipulator(robot.head)

def initialize(env_path='environments/pr_kitchen.robot.xml',
               robot_path='robots/herb2_padded.robot.xml',
               robot_name='HERB2', attach_viewer=False,
               **kw_args):
    env = openravepy.Environment()
    env.Load(env_path)

    robot = env.ReadRobotXMLFile(robot_path)
    robot.SetName(robot_name)
    env.Add(robot)

    if attach_viewer:
        env.SetViewer('qtcoin')

    initialize_herb(robot, **kw_args)
    return env, robot 

def initialize_sim(**kw_args):
    return initialize(left_arm_sim=True, right_arm_sim=True,
                      left_hand_sim=True, right_hand_sim=True,
                      head_sim=True, segway_sim=True, moped_sim=True,
                      **kw_args)
