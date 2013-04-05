import roslib; roslib.load_manifest('herbpy')
import openrave_exports; openrave_exports.export()
import rospkg, rospy
import atexit, functools, logging, numpy, signal, sys, types
import openravepy, manipulation2.trajectory, prrave.rave, or_multi_controller
import planner, herb, wam, yaml

NODE_NAME = 'herbpy'
OPENRAVE_FRAME_ID = '/openrave'
HEAD_NAMESPACE = '/head/owd'
LEFT_ARM_NAMESPACE = '/left/owd'
RIGHT_ARM_NAMESPACE = '/right/owd'
LEFT_HAND_NAMESPACE = '/left/bhd'
RIGHT_HAND_NAMESPACE = '/right/bhd'
MOPED_NAMESPACE = '/moped'
TALKER_NAMESPACE = '/talker'
rp = rospkg.RosPack()
herbpy_package_path = rp.get_path(NODE_NAME)

def initialize_logging():
    logger = logging.getLogger('herbpy')
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('{%(name)s:%(filename)s:%(lineno)d}:%(funcName)s: %(levelname)s - %(message)s')
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.INFO)
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger

logger = initialize_logging()

def attach_controller(robot, name, controller_args, dof_indices, affine_dofs, simulation):
    """
    Attach a controller to some of HERB DOFs. If in simulation, the specified
    controller is replaced with an IdealController.
    @param name controller's name within the multicontroller
    @param controller_args argument string used to construct the real controller
    @param dof_indices joint DOFs controlled by the controller
    @param affine_dofs affine DOFs controlled by the controller
    @param simulation flag for simulation mode
    @return controller
    """
    if simulation:
        controller_args = 'IdealController'

    delegate_controller = openravepy.RaveCreateController(robot.GetEnv(), controller_args)
    robot.multicontroller.attach(name, delegate_controller, dof_indices, affine_dofs)
    return delegate_controller

def initialize_manipulator(robot, manipulator, ik_type):
    """
    Bind extra methods to HERB's manipulators.
    @param manipulator one of HERB's manipulators
    @param ik_type inverse kinematics type for the manipulator
    """
    # Store a reference to the robot instance with extra bound methods.
    manipulator.parent = robot

    # Load the IK database.
    with robot.GetEnv():
        robot.SetActiveManipulator(manipulator)
        manipulator.ik_database = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=ik_type)
        if not manipulator.ik_database.load():
            logger.info('Generating IK database for {0:s}.'.format(manipulator.GetName()))
            manipulator.ik_database.autogenerate()

    # Bind extra methods for the manipulator.
    # TODO: Don't bind MoveHand for the head.
    wam.WamMethod.Bind(manipulator)

    # Dynamically wrap all of the planning functions such that they ignore the
    # active DOFs and plan using this manipulator.
    for method in planner.PlanningMethod.methods:
        def WrapPlan(method):
            @functools.wraps(method)
            def plan_method(manipulator, *args, **kw_args):
                with robot.CreateRobotStateSaver():
                    robot.SetActiveManipulator(manipulator)
                    robot.SetActiveDOFs(manipulator.GetArmIndices())
                    return getattr(robot, method.__name__)(*args, **kw_args)

            return plan_method

        bound_method = types.MethodType(WrapPlan(method), manipulator, type(manipulator))
        setattr(manipulator, method.__name__, bound_method)

def initialize_controllers(robot, left_arm_sim, right_arm_sim, left_hand_sim, right_hand_sim,
                                  head_sim, segway_sim):
    """
    Initialize HERB's controllers.
    @param head_sim simulate the head
    @param left_arm_sim simulate the left arm 
    @param right_arm_sim simulate the right arm 
    @param left_hand_sim simulate the left hand
    @param right_hand_sim simulate the right hand
    @param segway_sim simulate the Segway
    """
    head_args = 'OWDController {0:s} {1:s}'.format(NODE_NAME, HEAD_NAMESPACE)
    left_arm_args = 'OWDController {0:s} {1:s}'.format(NODE_NAME, LEFT_ARM_NAMESPACE)
    right_arm_args = 'OWDController {0:s} {1:s}'.format(NODE_NAME, RIGHT_ARM_NAMESPACE)
    left_hand_args = 'BHController {0:s} {1:s}'.format(NODE_NAME, LEFT_HAND_NAMESPACE)
    right_hand_args = 'BHController {0:s} {1:s}'.format(NODE_NAME, RIGHT_HAND_NAMESPACE)
    base_args = 'SegwayController {0:s}'.format(NODE_NAME)

    # Create aliases for the manipulators.
    left_arm_dofs = robot.left_arm.GetArmIndices()
    right_arm_dofs = robot.right_arm.GetArmIndices()
    left_hand_dofs = sorted(robot.left_arm.GetChildDOFIndices())
    right_hand_dofs = sorted(robot.right_arm.GetChildDOFIndices())
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

    # Create the MacTrajectory retimer for OWD.
    robot.mac_retimer = openravepy.RaveCreatePlanner(robot.GetEnv(), 'MacRetimer')

def initialize_sensors(robot, left_ft_sim, right_ft_sim, left_hand_sim, right_hand_sim, moped_sim, talker_sim):
    """
    Initialize HERB's sensor plugins.
    @param left_ft_sim simulate the left force/torque sensor
    @param right_ft_sim simulate the right force/torque sensor
    @param moped_sim simulate MOPED
    """
    env = robot.GetEnv()

    # Force/torque sensors.
    # TODO: Move this into the manipulator initialization function.
    # TODO: Why is SetName missing for sensors in the Python bindings?
    if not left_ft_sim:
        robot.left_arm.ft_sensor = openravepy.RaveCreateSensor(env,
            'BarrettFTSensor {0:s} {1:s}'.format(NODE_NAME, LEFT_ARM_NAMESPACE))
        env.Add(robot.left_arm.ft_sensor, True)
        
    if not left_hand_sim:
        robot.left_arm.handstate_sensor = openravepy.RaveCreateSensor(env,
            'HandstateSensor {0:s} {1:s}'.format(NODE_NAME, LEFT_HAND_NAMESPACE))       
        env.Add(robot.left_arm.handstate_sensor, True)

    if not right_ft_sim:
        robot.right_arm.ft_sensor = openravepy.RaveCreateSensor(env,
            'BarrettFTSensor {0:s} {1:s}'.format(NODE_NAME, RIGHT_ARM_NAMESPACE))
        env.Add(robot.right_arm.ft_sensor, True)

    if not right_hand_sim:
        robot.right_arm.handstate_sensor = openravepy.RaveCreateSensor(env,
            'HandstateSensor {0:s} {1:s}'.format(NODE_NAME, RIGHT_HAND_NAMESPACE))
        env.Add(robot.right_arm.handstate_sensor, True)

    # MOPED.
    if not moped_sim:
        moped_args = 'MOPEDSensorSystem {0:s} {1:s} {2:s}'.format(NODE_NAME, MOPED_NAMESPACE, OPENRAVE_FRAME_ID)
        robot.moped_sensorsystem = openravepy.RaveCreateSensorSystem(env, moped_args)

    # Talker.
    if not talker_sim:
        talker_args = 'TalkerModule {0:s} {1:s}'.format(NODE_NAME, TALKER_NAMESPACE)
        robot.talker_module = openravepy.RaveCreateModule(env, talker_args)

def initialize_herb(robot, left_arm_sim=True, right_arm_sim=True,
                           left_hand_sim=True, right_hand_sim=True,
                           head_sim=True, segway_sim=True,
                           left_ft_sim=True, right_ft_sim=True,
                           moped_sim=True, talker_sim=True,
                           **kw_args):
    """
    Bind extra methods to HERB.
    @param head_sim simulate the head
    @param left_arm_sim simulate the left arm 
    @param right_arm_sim simulate the right arm 
    @param left_hand_sim simulate the left hand
    @param right_hand_sim simulate the right hand
    @param left_ft_sim simulate the left force/torque sensor
    @param right_ft_sim simulate the right force/torque sensor
    @param segway_sim simulate the Segway
    @param moped_sim simulate MOPED
    @param talker_sim simulate talker
    """
    robot.left_arm = robot.GetManipulator('left_wam')
    robot.right_arm = robot.GetManipulator('right_wam')
    robot.head = robot.GetManipulator('head_wam')
    robot.manipulators = [ robot.left_arm, robot.right_arm, robot.head ]

    # Simulation flags.
    robot.left_arm_sim = left_arm_sim 
    robot.left_hand_sim = left_hand_sim 
    robot.left_ft_sim = left_ft_sim 
    robot.right_arm_sim = right_arm_sim 
    robot.right_hand_sim = right_hand_sim 
    robot.right_ft_sim = right_ft_sim 
    robot.head_sim = head_sim 
    robot.segway_sim = segway_sim
    robot.moped_sim = moped_sim

    # Initialize the OpenRAVE plugins.
    initialize_controllers(robot, left_arm_sim=left_arm_sim, right_arm_sim=right_arm_sim,
                                  left_hand_sim=left_hand_sim, right_hand_sim=right_hand_sim,
                                  head_sim=head_sim, segway_sim=segway_sim)
    initialize_sensors(robot, left_ft_sim=left_ft_sim, right_ft_sim=right_ft_sim, 
                       left_hand_sim=left_hand_sim, right_hand_sim=right_hand_sim,
                       moped_sim=moped_sim, talker_sim=talker_sim)

    # Wait for the robot's state to update.
    for controller in robot.controllers:
        try:
            controller.SendCommand('WaitForUpdate')
        except openravepy.openrave_exception, e:
            pass

    # Configure the planners. This order is specifically tuned for quickly
    # planning movehandstraight trajectories.
    import planner.chomp, planner.cbirrt, planner.jacobian, planner.mk, planner.snap
    robot.snap_planner = planner.snap.SnapPlanner(robot)
    robot.cbirrt_planner = planner.cbirrt.CBiRRTPlanner(robot)
    robot.chomp_planner = planner.chomp.CHOMPPlanner(robot)
    robot.mk_planner = planner.mk.MKPlanner(robot)
    robot.jacobian_planner = planner.jacobian.JacobianPlanner(robot)
    robot.planners = [ robot.snap_planner, robot.jacobian_planner, robot.mk_planner,
                       robot.chomp_planner, robot.cbirrt_planner  ]

    # Trajectory blending module.
    robot.trajectory_module = prrave.rave.load_module(robot.GetEnv(), 'Trajectory', robot.GetName())
    manipulation2.trajectory.bind(robot.trajectory_module)

    # Bind extra methods onto the OpenRAVE robot and manipulators..
    herb.HerbMethod.Bind(robot)

    # Dynamically bind the planners to the robot through the PlanGeneric wrapper.
    for method in planner.PlanningMethod.methods:
        # Wrapping this in a factory function is necessary to create a new
        # scope for the plan_method function. Otherwise the function would be
        # overwritten in subsequent loop iterations.
        def WrapPlan(method):
            @functools.wraps(method)
            def plan_method(robot, *args, **kw_args):
                return herb.PlanGeneric(robot, method.__name__, args, **kw_args)

            return plan_method

        bound_method = types.MethodType(WrapPlan(method), robot, type(robot))
        setattr(robot, method.__name__, bound_method)

    # Bind extra methods to the manipulators.
    initialize_manipulator(robot, robot.left_arm, openravepy.IkParameterization.Type.Transform6D)
    initialize_manipulator(robot, robot.right_arm, openravepy.IkParameterization.Type.Transform6D)
    initialize_manipulator(robot, robot.head, openravepy.IkParameterizationType.Lookat3D)

    # TODO: Enable the servo simulation after we diagnose the threading issues.
    '''
    if head_sim:
        robot.head.StartServoSim()
    if right_arm_sim:
        robot.right_arm.StartServoSim()
    if left_arm_sim:
        robot.left_arm.StartServoSim()
    '''

    # Convienence simulation flags for the manipulators.
    # TODO: Can we make a cleaner API for this?
    robot.left_arm.arm_simulated = left_arm_sim
    robot.left_arm.hand_simulated = left_hand_sim
    robot.left_arm.ft_simulated = left_ft_sim
    robot.right_arm.arm_simulated = right_arm_sim
    robot.right_arm.hand_simulated = right_hand_sim
    robot.right_arm.ft_simulated = right_ft_sim
    robot.head.arm_simulated = head_sim 

    # Set the default velocity and acceleration limits.
    # TODO: Move these constants into the robot's XML file.
    min_accel_time = 0.75
    max_jerk = 10 * numpy.pi
    head_velocity_limits = numpy.array([ 1.0, 1.0 ])
    arm_velocity_limits = numpy.array([ 0.75, 0.75, 2.0, 2.0, 2.5, 2.5, 2.5 ])
    robot.right_arm.SetVelocityLimits(arm_velocity_limits, min_accel_time)
    robot.left_arm.SetVelocityLimits(arm_velocity_limits, min_accel_time)
    robot.head.SetVelocityLimits(head_velocity_limits, min_accel_time)

    # Load saved configs
    initialize_saved_configs(robot, **kw_args)
    
def initialize_saved_configs(robot, yaml_path=None):
    if yaml_path is None:
        yaml_path = '%s/config/herb_robot_configs.yaml' % herbpy_package_path

    try:
        with open(yaml_path,'r') as yaml_file:
            robot.configs = {}
            robot_configs_dict = yaml.load(yaml_file);
            if 'named_indices' not in robot_configs_dict:
                raise Exception('yaml file does not include required \'named_indices\' entry'%yaml_path)
            named_inds = robot_configs_dict['named_indices']
            for config_name, config in robot_configs_dict.items():
                if config_name == 'named_indices':
                    continue
                robot.configs[config_name] = {}
                config_dofs = []
                config_vals = []
                for inds_name, vals in config.items():
                    if inds_name not in named_inds:
                        raise Exception('indices name \'%s\' is not in the \'named_indices\' dictionary'%inds_name)
                    config_dofs.extend( named_inds[inds_name] )
                    config_vals.extend( vals )
                    if len(config_dofs) != len(set(config_dofs)):
                        raise Exception('robot config \'%s\' has repeat dof indices: %s'%(config_name,str(config_dofs)))
                    sorted_config_dofs = sorted(config_dofs)
                    sorted_config_vals = [ config_vals[config_dofs.index(d)] for d in sorted_config_dofs ]
                    if len(config_dofs) > 0:
                        robot.configs[config_name]['dofs'] = numpy.array( sorted_config_dofs )
                        robot.configs[config_name]['vals'] = numpy.array( sorted_config_vals )
                    else:
                        del robot.configs[config_name]
    except Exception as e:
        raise Exception( 'initialize_saved_configs: Caught exception while loading yaml file \'%s\': %s'%(yaml_path, str(e)) )

def initialize(env_path='environments/pr_kitchen.robot.xml',
               robot_path='robots/herb2_padded.robot.xml',
               attach_viewer=False, **kw_args):
    """
    Load an environment, HERB to it, and optionally create a viewer. This
    accepts the same named parameters as initialize_herb.
    @param env_path path to the environment XML file
    @param robot_path path to the robot XML file
    @param attach_viewer attach a graphical viewer
    @param **kw_args named parameters for initialize_herb
    @return environment,robot
    """
    env = openravepy.Environment()
    env.Load(env_path)
    robot = env.ReadRobotXMLFile(robot_path)
    env.Add(robot)

    if attach_viewer:
        env.SetViewer('qtcoin')

    # Prevent ROS from intercepting Control+C.
    def RaiseKeyboardInterrupt(number, stack_frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, RaiseKeyboardInterrupt)

    #
    def HandleExit():
        env.Destroy()
        openravepy.RaveDestroy()
        rospy.signal_shutdown('herbpy is shutting down')
        sys.exit(0)
    atexit.register(HandleExit)

    initialize_herb(robot, **kw_args)
    return env, robot 

def initialize_sim(left_arm_sim=True, right_arm_sim=True,
                   left_hand_sim=True, right_hand_sim=True,
                   head_sim=True, segway_sim=True,
                   left_ft_sim=True, right_ft_sim=True,
                   moped_sim=True, talker_sim=True, **kw_args):
    """
    Initialize a simulated HERB. This is a convenience function that is simply
    a thin wrapper around initialize.
    @param **kw_args named parameters for initialize or initialize_herb
    """
    return initialize(left_arm_sim=left_arm_sim, right_arm_sim=right_arm_sim,
                      left_hand_sim=left_hand_sim, right_hand_sim=right_hand_sim,
                      head_sim=head_sim, segway_sim=segway_sim,
                      left_ft_sim=left_ft_sim, right_ft_sim=right_ft_sim,
                      moped_sim=moped_sim, talker_sim=talker_sim, **kw_args)

def initialize_real(left_arm_sim=False, right_arm_sim=False,
                    left_hand_sim=False, right_hand_sim=False,
                    head_sim=False, segway_sim=False,
                    left_ft_sim=False, right_ft_sim=False,
                    moped_sim=False, talker_sim=False, **kw_args):
    """
    Initialize the real HERB. This is a convenience function that is simply a
    thin wrapper around initialize.
    @param **kw_args named parameters for initialize or initialize_herb
    """
    return initialize(left_arm_sim=left_arm_sim, right_arm_sim=right_arm_sim,
                      left_hand_sim=left_hand_sim, right_hand_sim=right_hand_sim,
                      head_sim=head_sim, segway_sim=segway_sim,
                      left_ft_sim=left_ft_sim, right_ft_sim=right_ft_sim,
                      moped_sim=moped_sim, talker_sim=talker_sim, **kw_args)
