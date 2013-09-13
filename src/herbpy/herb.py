PACKAGE = 'herbpy'
import roslib; roslib.load_manifest(PACKAGE)
import openrave_exports; openrave_exports.export(PACKAGE)
import prpy, openravepy

# Add dependencies to our OpenRAVE plugin and data search paths.
import prpy.dependency_manager
prpy.dependency_manager.export(PACKAGE)

def initialize(robot_xml='robots/herb2_padded_nosensors.robot.xml',
               attach_viewer=False, sim=True, **kw_args):
    # Create the environment.
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile(robot_xml)
    env.Add(robot)
    prpy.logger.initialize_logging()

    # Default arguments.
    keys = [ 'left_arm_sim', 'left_hand_sim', 'left_ft_sim',
             'right_arm_sim', 'right_hand_sim', 'right_ft_sim',
             'head_sim', 'moped_sim', 'talker_sim' ]
    for key in keys:
        if key not in kw_args:
            kw_args[key] = sim

    from herbrobot import HERBRobot
    prpy.bind_subclass(robot, HERBRobot, **kw_args)

    if attach_viewer:
        env.SetViewer('qtcoin')

    return env, robot
