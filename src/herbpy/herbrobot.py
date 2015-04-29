PACKAGE = 'herbpy'
import logging
import openravepy
import prpy
from prpy.base.barretthand import BarrettHand
from prpy.base.wam import WAM
from prpy.base.wamrobot import WAMRobot
from prpy.exceptions import PrPyException
from prpy.planning.base import UnsupportedPlanningError
from herbbase import HerbBase
from herbpantilt import HERBPantilt

logger = logging.getLogger('herbpy')


def try_and_warn(fn, exception_type, message, default_value=None):
    try:
        return fn()
    except exception_type:
        logger.warning(message)
        return None


class HERBRobot(WAMRobot):
    def __init__(self, left_arm_sim, right_arm_sim, right_ft_sim,
                       left_hand_sim, right_hand_sim, left_ft_sim,
                       head_sim, talker_sim, segway_sim):
        WAMRobot.__init__(self, robot_name='herb')

        # Absolute path to this package.
        from rospkg import RosPack
        ros_pack = RosPack()
        package_path = ros_pack.get_path(PACKAGE)

        # Convenience attributes for accessing self components.
        self.left_arm = self.GetManipulator('left')
        self.right_arm = self.GetManipulator('right')
        self.head = self.GetManipulator('head')
        self.left_arm.hand = self.left_arm.GetEndEffector()
        self.right_arm.hand = self.right_arm.GetEndEffector()
        self.left_hand = self.left_arm.hand
        self.right_hand = self.right_arm.hand
        self.manipulators = [ self.left_arm, self.right_arm, self.head ]

        # Dynamically switch to self-specific subclasses.
        prpy.bind_subclass(self.left_arm, WAM, sim=left_arm_sim, owd_namespace='/left/owd')
        prpy.bind_subclass(self.right_arm, WAM, sim=right_arm_sim, owd_namespace='/right/owd')
        prpy.bind_subclass(self.head, HERBPantilt, sim=head_sim, owd_namespace='/head/owd')
        prpy.bind_subclass(self.left_arm.hand, BarrettHand, sim=left_hand_sim, manipulator=self.left_arm,
                           owd_namespace='/left/owd', bhd_namespace='/left/bhd', ft_sim=right_ft_sim)
        prpy.bind_subclass(self.right_arm.hand, BarrettHand, sim=right_hand_sim, manipulator=self.right_arm,
                           owd_namespace='/right/owd', bhd_namespace='/right/bhd', ft_sim=right_ft_sim)
        self.base = HerbBase(sim=segway_sim, robot=self)

        # Set HERB's acceleration limits. These are not specified in URDF.
        accel_limits = self.GetDOFAccelerationLimits()
        accel_limits[self.head.GetArmIndices()] = [ 2. ] * self.head.GetArmDOF()
        accel_limits[self.left_arm.GetArmIndices()] = [ 2. ] * self.left_arm.GetArmDOF()
        accel_limits[self.right_arm.GetArmIndices()] = [ 2. ] * self.right_arm.GetArmDOF()
        self.SetDOFAccelerationLimits(accel_limits)
        
        # Support for named configurations.
        import os.path
        self.configurations.add_group('left_arm', self.left_arm.GetArmIndices())
        self.configurations.add_group('right_arm', self.right_arm.GetArmIndices())
        self.configurations.add_group('head', self.head.GetArmIndices())
        self.configurations.add_group('left_hand', self.left_hand.GetIndices())
        self.configurations.add_group('right_hand', self.right_hand.GetIndices())

        if prpy.dependency_manager.is_catkin():
            from catkin.find_in_workspaces import find_in_workspaces
            configurations_paths = find_in_workspaces(search_dirs=['share'], project='herbpy',
                    path='config/configurations.yaml', first_match_only=True)
            if not configurations_paths:
                raise ValueError('Unable to load named configurations from "config/configurations.yaml".')

            configurations_path = configurations_paths[0]
        else:
            configurations_path = os.path.join(package_path, 'config/configurations.yaml')

        try:
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            raise ValueError('Failed laoding named configurations from "{:s}".'.format(
                configurations_path))

        # Load default TSRs from YAML.
        if self.tsrlibrary is not None:
            if prpy.dependency_manager.is_catkin():
                from catkin.find_in_workspaces import find_in_workspaces
                tsrs_paths = find_in_workspaces(search_dirs=['share'], project='herbpy',
                                 path='config/tsrs.yaml', first_match_only=True)
                if not tsrs_paths:
                    logger.info('Unable to load named tsrs from "config/tsrs.yaml".')
                    tsrs_path = None
                else:
                    tsrs_path = tsrs_paths[0]
            else:
                tsrs_path = os.path.join(package_path, 'config/tsrs.yaml')

            if tsrs_path is not None:
                try:
                    self.tsrlibrary.load_yaml(tsrs_path)
                except IOError as e:
                    raise ValueError('Failed loading named tsrs from "{:s}".'.format(
                            tsrs_path))

        # Initialize a default planning pipeline.
        from prpy.planning import (
            FirstSupported,
            MethodMask,
            Ranked,
            Sequence,
        )
        from prpy.planning import (
            BiRRTPlanner,
            CBiRRTPlanner,
            CHOMPPlanner,
            GreedyIKPlanner,
            IKPlanner,
            NamedPlanner,
            SBPLPlanner,
            SnapPlanner,
            TSRPlanner,
            VectorFieldPlanner
        )

        # TODO: These should be meta-planners.
        self.named_planner = NamedPlanner()
        self.ik_planner = IKPlanner()

        # Special-purpose planners.
        self.snap_planner = SnapPlanner()
        self.vectorfield_planner = VectorFieldPlanner()
        self.greedyik_planner = GreedyIKPlanner()

        # General-purpose planners.
        self.birrt_planner = BiRRTPlanner()
        self.cbirrt_planner = CBiRRTPlanner()

        # Trajectory optimizer.
        try:
            from or_trajopt import TrajoptPlanner

            self.trajopt_planner = TrajoptPlanner()
        except ImportError:
            self.trajopt_planner = None
            logger.warning('Failed creating TrajoptPlanner. Is the or_trajopt'
                           ' package in your workspace and built?')

        try:
            self.chomp_planner = CHOMPPlanner()
        except UnsupportedPlanningError:
            self.chomp_planner = None
            logger.warning('Failed loading the CHOMP module. Is the or_cdchomp'
                           ' package in your workspace and built?')

        if not (self.trajopt_planner or self.chomp_planner):
            raise PrPyException('Unable to load both CHOMP and TrajOpt. At'
                                ' least one of these packages is required.')
        
        actual_planner = Sequence(
            # First, try the straight-line trajectory.
            self.snap_planner,
            # Then, try a few simple (and fast!) heuristics.
            self.vectorfield_planner,
            self.greedyik_planner,
            # Next, try a trajectory optimizer.
            self.trajopt_planner or self.chomp_planner
        )
        self.planner = FirstSupported(
            Sequence(actual_planner, 
                     TSRPlanner(delegate_planner=actual_planner),
                     self.cbirrt_planner),
            # Special purpose meta-planner.
            NamedPlanner(delegate_planner=actual_planner),
        )
        
        from prpy.planning.retimer import HauserParabolicSmoother
        self.smoother = HauserParabolicSmoother()
        self.retimer = HauserParabolicSmoother()
        self.simplifier = None

        # Base planning
        if prpy.dependency_manager.is_catkin():
            from catkin.find_in_workspaces import find_in_workspaces
            planner_parameters_paths = find_in_workspaces(search_dirs=['share'], project='herbpy',
                    path='config/base_planner_parameters.yaml', first_match_only=True)
            if not planner_parameters_paths:
                raise ValueError(
                    'Unable to load base planner parameters from "config/base_planner_parameters.yaml".')

            planner_parameters_path = planner_parameters_paths[0]
        else:
            planner_parameters_path = os.path.join(package_path, 'config/base_planner_parameters.yaml')

        self.sbpl_planner = SBPLPlanner()
        try:
            with open(planner_parameters_path, 'rb') as config_file:
                import yaml
                params_yaml = yaml.load(config_file)
            self.sbpl_planner.SetPlannerParameters(params_yaml)
        except IOError as e:
            raise ValueError('Failed loading base planner parameters from "{:s}".'.format(
                planner_parameters_path))

        self.base_planner = self.sbpl_planner

        # Actions and TSRs
        from prpy.action import ActionLibrary
        self.actions = ActionLibrary()
        from herbpy.action import *
        from herbpy.tsr import *

        # Setting necessary sim flags
        self.talker_simulated = talker_sim
        self.segway_sim = segway_sim

    def CloneBindings(self, parent):
        from prpy import Cloned
        WAMRobot.CloneBindings(self, parent)
        self.left_arm = Cloned(parent.left_arm)
        self.right_arm = Cloned(parent.right_arm)
        self.head = Cloned(parent.head)
        self.left_arm.hand = Cloned(parent.left_arm.GetEndEffector())
        self.right_arm.hand = Cloned(parent.right_arm.GetEndEffector())
        self.left_hand = self.left_arm.hand
        self.right_hand = self.right_arm.hand
        self.manipulators = [ self.left_arm, self.right_arm, self.head ]
        self.planner = parent.planner
        self.base_planner = parent.base_planner

    def Say(robot, message):
        """Say a message using HERB's text-to-speech engine.
        @param message
        """
        from pr_msgs.srv import AppletCommand
        import rospy

        if not robot.talker_simulated:
            # XXX: HerbPy should not make direct service calls.
            logger.info('Saying "%s".', message)
            #rospy.wait_for_service('/talkerapplet')
            talk = rospy.ServiceProxy('/talkerapplet', AppletCommand)    
            try:
                talk('say', message, 0, 0)
            except rospy.ServiceException, e:
                logger.error('Error talking.')

    def SetStiffness(self, stiffness):
        """Set the stiffness of HERB's arms and head.
        Zero is gravity compensation, one is position control. Stifness values
        between zero and one are experimental.
        @param stiffness value between zero and one
        """
        self.head.SetStiffness(stiffness)
        self.left_arm.SetStiffness(stiffness)
        self.right_arm.SetStiffness(stiffness)

    def DriveStraightUntilForce(robot, direction, velocity=0.1, force_threshold=3.0,
                                max_distance=None, timeout=None, left_arm=True, right_arm=True):
        """Deprecated. Use base.DriveStraightUntilForce instead.
        """
        logger.warning('DriveStraightUntilForce is deprecated. Use base.DriveStraightUntilForce instead.')
        robot.base.DriveStraightUntilForce(direction, velocity, force_threshold,
                                max_distance, timeout, left_arm, right_arm)

    def DriveAlongVector(robot, direction, goal_pos):
        import numpy
        # TODO: Do we still need this? If so, we should move it into HERBBase.
        direction = direction[:2]/numpy.linalg.norm(direction[:2])
        herb_pose = robot.GetTransform()
        distance = numpy.dot(goal_pos[:2]-herb_pose[:2,3], direction)
        cur_angle = numpy.arctan2(herb_pose[1,0],herb_pose[0,0])
        des_angle = numpy.arctan2(direction[1],direction[0])
        robot.RotateSegway(des_angle-cur_angle)
        robot.DriveSegway(distance)

    def DriveSegway(robot, meters, **kw_args):
        """Deprecated. Use base.Forward instead.
        """
        logger.warning('DriveSegway is deprecated. Use base.Forward instead.')
        robot.base.Forward(meters, **kw_args)

    def DriveSegwayToNamedPosition(robot, named_position):
        """Deprecated. Use base.PlanToBasePose instead.
        """
        if robot.segway_sim:
            raise Exception('Driving to named positions is not supported in simulation.')
        else:
            robot.base.controller.SendCommand("Goto " + named_position)

    def RotateSegway(robot, angle_rad, **kw_args):
        """Deprecated. Use base.Rotate instead.
        """
        logger.warning('RotateSegway is deprecated. Use base.Rotate instead.')
        robot.base.Rotate(angle_rad, **kw_args)

    def StopSegway(robot):
        # TODO: This should be moved into HERBBase.
        if not robot.segway_sim:
            robot.base.controller.SendCommand("Stop")
