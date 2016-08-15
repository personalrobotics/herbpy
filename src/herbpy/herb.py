import json
import logging
import os
import prpy
import prpy.dependency_manager
from prpy.collision import (
    BakedRobotCollisionCheckerFactory,
    SimpleRobotCollisionCheckerFactory,
)
from openravepy import (
    Environment,
    RaveCreateModule,
    RaveCreateCollisionChecker,
    RaveInitialize,
    openrave_exception,
)
from .herbbase import HerbBase
from .herbrobot import HERBRobot

logger = logging.getLogger('herbpy')

def initialize(robot_xml=None, env_path=None, attach_viewer=False,
               sim=True, **kw_args):
    prpy.logger.initialize_logging()

    # Hide TrajOpt logging.
    os.environ.setdefault('TRAJOPT_LOG_THRESH', 'WARN')

    # Load plugins.
    prpy.dependency_manager.export()
    RaveInitialize(True)

    # Create the environment.
    env = Environment()
    if env_path is not None:
        if not env.Load(env_path):
            raise ValueError(
                'Unable to load environment from path {:s}'.format(env_path))

    herb_name = None
    # Load the URDF file into OpenRAVE.
    urdf_module = RaveCreateModule(env, 'urdf')
    if urdf_module is None:
        logger.error('Unable to load or_urdf module. Do you have or_urdf'
                     ' built and installed in one of your Catkin workspaces?')
        raise ValueError('Unable to load or_urdf plugin.')

    if sim:
        urdf_uri = 'package://herb_description/robots/herb.urdf'
        srdf_uri = 'package://herb_description/robots/herb.srdf'
        args = 'LoadURI {:s} {:s}'.format(urdf_uri, srdf_uri)
        herb_name = urdf_module.SendCommand(args)
    else:
        import rospy
        if not rospy.core.is_initialized():
            raise RuntimeError('rospy not initialized. '
                               'Must call rospy.init_node()')
        urdf_string = rospy.get_param('/robot_description', None)
        if urdf_string is None:
            raise RuntimeError('rosparam "/robot_description" is not set.'
                               ' Unable to load correct HERB model.')
        srdf_string = rospy.get_param('/semantic_robot_description', None)
        if srdf_string is None:
            raise RuntimeError('rosparam "/semantic_robot_description" is not'
                               ' set. Unable to load correct HERB model.')
        urdf_json_wrapper = json.dumps(
            {'urdf': urdf_string.replace('\n', ' ').replace('\r', ' '),
             'srdf': srdf_string.replace('\n', ' ').replace('\r', ' ')})
        args = 'LoadString {}'.format(urdf_json_wrapper)
        herb_name = urdf_module.SendCommand(args)

    if herb_name is None:
        raise ValueError('Failed loading HERB model using or_urdf.')

    robot = env.GetRobot(herb_name)
    if robot is None:
        raise ValueError('Unable to find robot with name "{:s}".'.format(
                         herb_name))

    # Default to FCL.
    collision_checker = RaveCreateCollisionChecker(env, 'fcl')
    if collision_checker is not None:
        env.SetCollisionChecker(collision_checker)
    else:
        collision_checker = env.GetCollisionChecker()
        logger.warning(
            'Failed creating "fcl", defaulting to the default OpenRAVE'
            ' collision checker. Did you install or_fcl?')

    # Enable baking if it is supported.
    try:
        result = collision_checker.SendCommand('BakeGetType')
        is_baking_suported = (result is not None)
    except openrave_exception:
        is_baking_suported = False

    if is_baking_suported:
        robot_checker_factory = BakedRobotCollisionCheckerFactory()
    else:
        robot_checker_factory = SimpleRobotCollisionCheckerFactory()
        logger.warning(
            'Collision checker does not support baking. Defaulting to'
            ' the slower SimpleRobotCollisionCheckerFactory.')

    # Default arguments.
    keys = [ 'left_arm_sim', 'left_hand_sim', 'left_ft_sim',
             'right_arm_sim', 'right_hand_sim', 'right_ft_sim',
             'head_sim', 'talker_sim', 'segway_sim', 'perception_sim' ]
    for key in keys:
        if key not in kw_args:
            kw_args[key] = sim

    prpy.bind_subclass(robot, HERBRobot,
        robot_checker_factory=robot_checker_factory, **kw_args)

    if sim:
        dof_indices, dof_values \
            = robot.configurations.get_configuration('relaxed_home')
        robot.SetDOFValues(dof_values, dof_indices)

    # Start by attempting to load or_rviz.
    if attach_viewer == True:
        attach_viewer = 'rviz'
        env.SetViewer(attach_viewer)

        # Fall back on qtcoin if loading or_rviz failed
        if env.GetViewer() is None:
            logger.warning(
                'Loading the RViz viewer failed. Do you have or_interactive'
                ' marker installed? Falling back on qtcoin.')
            attach_viewer = 'qtcoin'

    if attach_viewer and env.GetViewer() is None:
        env.SetViewer(attach_viewer)
        if env.GetViewer() is None:
            raise Exception('Failed creating viewer of type "{0:s}".'.format(
                            attach_viewer))

    # Remove the ROS logging handler again. It might have been added when we
    # loaded or_rviz.
    prpy.logger.remove_ros_logger()

    return env, robot
