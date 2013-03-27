import roslib; roslib.load_manifest('herbpy')
import openrave_exports; openrave_exports.export()
import logging, types
import openravepy, manipulation2.trajectory, prrave.rave, or_multi_controller
import cbirrt, chomp, herb, wam, yaml

NODE_NAME = 'herbpy'
OPENRAVE_FRAME_ID = '/openrave'

def attach_controller(robot, name, controller_args, dof_indices, affine_dofs, simulation):
    if simulation:
        controller_args = 'IdealController'

    delegate_controller = openravepy.RaveCreateController(robot.GetEnv(), controller_args)
    robot.multicontroller.attach(name, delegate_controller, dof_indices, affine_dofs)
    return delegate_controller

def initialize_manipulator(robot, manipulator, ik_type):
    # Load the IK database.
    with robot.GetEnv():
        robot.SetActiveManipulator(manipulator)
        manipulator.ik_database = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=ik_type)
        if not manipulator.ik_database.load():
            logging.info('Generating IK database for {0:s}.'.format(manipulator.GetName()))
            manipulator.ik_database.autogenerate()

    # Bind extra methods.
    t = type(manipulator)
    manipulator.parent = robot
    manipulator.SetStiffness = types.MethodType(wam.SetStiffness, manipulator, t)
    manipulator.MoveHand = types.MethodType(wam.MoveHand, manipulator, t)

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
    head_dofs = robot.head.GetArmIndices()

    # Controllers.
    robot.multicontroller = or_multi_controller.MultiControllerWrapper(robot)
    robot.head.arm_controller = attach_controller(robot, 'head', head_args, head_dofs, 0, head_sim)
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
        self.moped_sensorsystem = openravepy.RaveCreateSensorSystem(robot.GetEnv(), args)

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

    # Trajectory blending module.
    robot.trajectory_module = prrave.rave.load_module(robot.GetEnv(), 'Trajectory', robot.GetName())
    manipulation2.trajectory.bind(robot.trajectory_module)

    # Bind extra methods to the manipulators.
    initialize_manipulator(robot, robot.left_arm, openravepy.IkParameterization.Type.Transform6D)
    initialize_manipulator(robot, robot.right_arm, openravepy.IkParameterization.Type.Transform6D)
    initialize_manipulator(robot, robot.head, openravepy.IkParameterizationType.Lookat3D)

    # Bind extra methods onto the OpenRAVE robot.
    t = type(robot)
    robot.PlanToConfiguration = types.MethodType(herb.PlanToConfiguration, robot, t)
    robot.PlanToEndEffectorPose = types.MethodType(herb.PlanToEndEffectorPose, robot, t)
    robot.BlendTrajectory = types.MethodType(herb.BlendTrajectory, robot, t)
    robot.ExecuteTrajectory = types.MethodType(herb.ExecuteTrajectory, robot, t)
    robot.AddTrajectoryFlags = types.MethodType(herb.AddTrajectoryFlags, robot, t)
    robot.LookAt = types.MethodType(herb.LookAt, robot, t)

def initialize(env_path='environments/pr_kitchen.robot.xml',
               robot_path='robots/herb2_padded.robot.xml',
               attach_viewer=False, **kw_args):
    env = openravepy.Environment()
    env.Load(env_path)

    robot = env.ReadRobotXMLFile(robot_path)
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
