PACKAGE = 'herbpy'
import roslib; roslib.load_manifest(PACKAGE)
import logging, prpy, openravepy
from herbbase import HerbBase

logger = logging.getLogger('herbpy')

# Add dependencies to our OpenRAVE plugin and data search paths.
import prpy.dependency_manager
prpy.dependency_manager.export(PACKAGE)

def initialize(robot_xml=None, env_path=None, attach_viewer=False, sim=True, **kw_args):
    # Create the environment.
    env = openravepy.Environment()
    if env_path is not None:
        if not env.Load(env_path):
            raise Exception('Unable to load environment frompath %s' % env_path)

    if robot_xml is None:
        import os, rospkg
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('herb_description')
        robot_xml = os.path.join(base_path, 'ordata', 'robots', 'herb.robot.xml')

    robot = env.ReadRobotXMLFile(robot_xml)
    env.Add(robot)
    prpy.logger.initialize_logging()

    # Default arguments.
    keys = [ 'left_arm_sim', 'left_hand_sim', 'left_ft_sim',
             'right_arm_sim', 'right_hand_sim', 'right_ft_sim',
             'head_sim', 'moped_sim', 'talker_sim', 'segway_sim' ]
    for key in keys:
        if key not in kw_args:
            kw_args[key] = sim

    from herbrobot import HERBRobot
    prpy.bind_subclass(robot, HERBRobot, **kw_args)

    # Start by attempting to load or_rviz.
    if attach_viewer == True:
        attach_viewer = 'or_rviz'
        env.SetViewer(attach_viewer)

        # Fall back on qtcoin if loading or_rviz failed
        if env.GetViewer() is None:
            logger.warning('Loading or_rviz failed. Falling back on qt_coin.')
            attach_viewer = 'qtcoin'

    if attach_viewer and env.GetViewer() is None:
        env.SetViewer(attach_viewer)
        if env.GetViewer() is None:
            raise Exception('Failed creating viewer of type "{0:s}".'.format(attach_viewer))

    return env, robot
