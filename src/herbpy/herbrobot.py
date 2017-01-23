PACKAGE = 'herbpy'
import logging
import numbers
import prpy
import prpy.rave
import prpy.util
import yaml
import subprocess
from .barretthand import BarrettHand
from .herbbase import HerbBase
from .herbpantilt import HERBPantilt
from .wam import WAM
from prpy import Cloned
from prpy.action import ActionLibrary
from prpy.base.robot import Robot
from prpy.controllers import RewdOrTrajectoryController
from prpy.exceptions import TrajectoryNotExecutable
from prpy.named_config import ConfigurationLibrary
from prpy.planning import (
    CBiRRTPlanner,
    FirstSupported,
    NamedPlanner,
    SBPLPlanner,
    Sequence,
    SnapPlanner,
    TSRPlanner,
    OMPLPlanner,
    VectorFieldPlanner,
)
from or_trajopt import TrajoptPlanner
from prpy.planning.retimer import HauserParabolicSmoother
from prpy.util import FindCatkinResource


logger = logging.getLogger('herbpy')


def try_and_warn(fn, exception_type, message, default_value=None):
    try:
        return fn()
    except exception_type:
        logger.warning(message)
        return None


class HERBRobot(Robot):
    def __init__(self, left_arm_sim, right_arm_sim, right_ft_sim,
                       left_hand_sim, right_hand_sim, left_ft_sim,
                       head_sim, talker_sim, segway_sim, perception_sim,
                       robot_checker_factory):
        Robot.__init__(self, robot_name='herb')
        self.robot_checker_factory = robot_checker_factory

        # Controller setup
        self.controller_manager = None
        self.controllers_always_on = []

        self.full_controller_sim = (left_arm_sim and right_arm_sim and
                                    left_ft_sim and right_ft_sim and
                                    left_hand_sim and right_hand_sim and
                                    head_sim)
        if not self.full_controller_sim:
            # any non-simulation requires ros and the ros_control stack
            import rospy
            from ros_control_client_py import (
                ControllerManagerClient,
                JointStateClient,
            )

            if not rospy.core.is_initialized():
                raise RuntimeError('rospy not initialized. '
                                   'Must call rospy.init_node()')

            # update openrave state from /joint_states
            self._jointstate_client = JointStateClient(
                self, topic_name='/joint_states')

            self.controller_manager = ControllerManagerClient()
            self.controllers_always_on.append('joint_state_controller')

        # Convenience attributes for accessing self components.
        self.left_arm = self.GetManipulator('left')
        self.right_arm = self.GetManipulator('right')
        self.head = self.GetManipulator('head')
        self.left_arm.hand = self.left_arm.GetEndEffector()
        self.right_arm.hand = self.right_arm.GetEndEffector()
        self.left_hand = self.left_arm.hand
        self.right_hand = self.right_arm.hand
        self.manipulators = [self.left_arm, self.right_arm, self.head]

        # Dynamically switch to self-specific subclasses.
        prpy.bind_subclass(self.left_arm, WAM, sim=left_arm_sim, namespace='/left')
        prpy.bind_subclass(self.right_arm, WAM, sim=right_arm_sim, namespace='/right')
        prpy.bind_subclass(self.head, HERBPantilt, sim=head_sim, owd_namespace='/head/owd')
        prpy.bind_subclass(self.left_arm.hand, BarrettHand, sim=left_hand_sim, manipulator=self.left_arm,
                           bhd_namespace='/left', ft_sim=left_ft_sim)
        prpy.bind_subclass(self.right_arm.hand, BarrettHand, sim=right_hand_sim, manipulator=self.right_arm,
                           bhd_namespace='/right', ft_sim=right_ft_sim)
        self.base = HerbBase(sim=segway_sim, robot=self)

        # Set HERB's acceleration limits. These are not specified in URDF.
        accel_limits = self.GetDOFAccelerationLimits()
        accel_limits[self.head.GetArmIndices()] = [2.] * self.head.GetArmDOF()
        accel_limits[self.left_arm.GetArmIndices()] = [2.] * self.left_arm.GetArmDOF()
        accel_limits[self.right_arm.GetArmIndices()] = [2.] * self.right_arm.GetArmDOF()
        self.SetDOFAccelerationLimits(accel_limits)


        # Determine always-on controllers

        # hand controllers
        if not left_hand_sim:
            self.controllers_always_on.append('left_hand_controller')

        if not right_hand_sim:
            self.controllers_always_on.append('right_hand_controller')

        # force/torque controllers
        if not left_ft_sim or not right_ft_sim:
            self.controllers_always_on.append('force_torque_sensor_controller')

        if not left_ft_sim:
            self.controllers_always_on.append('left_tare_controller')

        if not right_ft_sim:
            self.controllers_always_on.append('right_tare_controller')

        # Set default manipulator controllers in sim only
        # NOTE: head is ignored until TODO new Schunk head integrated
        if left_arm_sim:
            self.left_arm.sim_controller = self.AttachController(name=self.left_arm.GetName(),
                                                                 args='IdealController',
                                                                 dof_indices=self.left_arm.GetArmIndices(),
                                                                 affine_dofs=0,
                                                                 simulated=True)

        if right_arm_sim:
            self.right_arm.sim_controller = self.AttachController(name=self.right_arm.GetName(),
                                                                  args='IdealController',
                                                                  dof_indices=self.right_arm.GetArmIndices(),
                                                                  affine_dofs=0,
                                                                  simulated=True)

        # load and activate initial controllers
        if self.controller_manager is not None:
            self.controller_manager.request(
                self.controllers_always_on).switch()

        # Support for named configurations.
        import os.path
        self.configurations.add_group('left_arm', self.left_arm.GetArmIndices())
        self.configurations.add_group('right_arm', self.right_arm.GetArmIndices())
        self.configurations.add_group('head', self.head.GetArmIndices())
        self.configurations.add_group('left_hand', self.left_hand.GetIndices())
        self.configurations.add_group('right_hand', self.right_hand.GetIndices())

        configurations_path = FindCatkinResource('herbpy', 'config/configurations.yaml')

        try:
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            raise ValueError('Failed laoding named configurations from "{:s}".'.format(
                configurations_path))

        # Hand configurations
        for hand in [self.left_hand, self.right_hand]:
            hand.configurations = ConfigurationLibrary()
            hand.configurations.add_group('hand', hand.GetIndices())

            if isinstance(hand, BarrettHand):
                hand_configs_path = FindCatkinResource('herbpy', 'config/barrett_preshapes.yaml')
                try:
                    hand.configurations.load_yaml(hand_configs_path)
                except IOError as e:
                    raise ValueError('Failed loading named hand configurations from "{:s}".'.format(
                        hand_configs_path))
            else:
                logger.warning('Unrecognized hand class. Not loading named configurations.')

        # Planner.
        snap_planner = SnapPlanner(
            robot_checker_factory=self.robot_checker_factory)
        vectorfield_planner = VectorFieldPlanner(
            robot_checker_factory=self.robot_checker_factory)
        trajopt_planner = TrajoptPlanner(
            robot_checker_factory=self.robot_checker_factory)
        rrt_planner = OMPLPlanner('RRTConnect',
            robot_checker_factory=self.robot_checker_factory)
        cbirrt_planner = CBiRRTPlanner(
            timelimit=1.,
            robot_checker_factory=self.robot_checker_factory)

        actual_planner = Sequence(
            snap_planner,
            vectorfield_planner,
            trajopt_planner,
            TSRPlanner(
                delegate_planner=Sequence(snap_planner, trajopt_planner),
                robot_checker_factory=self.robot_checker_factory),
            FirstSupported(
                rrt_planner,
                cbirrt_planner),
        )

        self.planner = FirstSupported(
            actual_planner,
            NamedPlanner(delegate_planner=actual_planner),
        )

        # Post-processor.
        self.smoother = HauserParabolicSmoother(
            do_blend=True, blend_iterations=1, blend_radius=0.4,
            do_shortcut=True, timelimit=0.6)
        self.retimer = HauserParabolicSmoother(
            do_blend=True, blend_iterations=1, blend_radius=0.4,
            do_shortcut=False)
        self.simplifier = None

        # Base planning
        planner_parameters_path = FindCatkinResource('herbpy', 'config/base_planner_parameters.yaml')

        self.sbpl_planner = SBPLPlanner()
        try:
            with open(planner_parameters_path, 'rb') as config_file:
                params_yaml = yaml.load(config_file)
            self.sbpl_planner.SetPlannerParameters(params_yaml)
        except IOError as e:
            raise ValueError('Failed loading base planner parameters from "{:s}".'.format(
                planner_parameters_path))

        self.base_planner = self.sbpl_planner

        # Create action library
        self.actions = ActionLibrary()

        # Register default actions and TSRs
        import herbpy.action
        import herbpy.tsr

        # Setting necessary sim flags
        self.talker_simulated = talker_sim
        self.segway_sim = segway_sim

        # Set up perception
        self.detector = None
        if perception_sim:
            from prpy.perception import SimulatedPerceptionModule
            self.detector = SimulatedPerceptionModule()
        else:
            from prpy.perception import ApriltagsModule
            try:
                kinbody_path = FindCatkinResource('pr_ordata',
                                                            'data/objects')
                marker_data_path = FindCatkinResource('pr_ordata',
                                                                'data/objects/tag_data.json')
                self.detector = ApriltagsModule(marker_topic='/apriltags_kinect2/marker_array',
                                                marker_data_path=marker_data_path,
                                                kinbody_path=kinbody_path,
                                                detection_frame='head/kinect2_rgb_optical_frame',
                                                destination_frame='herb_base',
                                                reference_link=self.GetLink('/herb_base'))
            except IOError as e:
                logger.warning('Failed to find required resource path. ' \
                               'pr_ordata package cannot be found. ' \
                               'Perception detector will not be loaded.' \
                               '\n{}'.format(e))

        if not self.talker_simulated:
            # Initialize herbpy ROS Node
            import rospy
            if not rospy.core.is_initialized():
                raise RuntimeError('rospy not initialized. '
                                   'Must call rospy.init_node()')

            import talker.msg
            from actionlib import SimpleActionClient
            self._say_action_client = SimpleActionClient('say', talker.msg.SayAction)

    def CloneBindings(self, parent):
        super(HERBRobot, self).CloneBindings(parent)
        self.left_arm = Cloned(parent.left_arm)
        self.right_arm = Cloned(parent.right_arm)
        self.head = Cloned(parent.head)
        self.left_arm.hand = Cloned(parent.left_arm.GetEndEffector())
        self.right_arm.hand = Cloned(parent.right_arm.GetEndEffector())
        self.left_hand = self.left_arm.hand
        self.right_hand = self.right_arm.hand
        self.manipulators = [self.left_arm, self.right_arm, self.head]
        self.planner = parent.planner
        self.base_planner = parent.base_planner

    def _ExecuteTrajectory(self, traj, defer=False, timeout=None, period=0.01,
                           **kwargs):
        if defer is not False:
            raise RuntimeError('defer functionality was deprecated in '
                               'personalrobotics/prpy#278')
        # Don't execute trajectories that don't have at least one waypoint.
        if traj.GetNumWaypoints() <= 0:
            raise ValueError('Trajectory must contain at least one waypoint.')

        # Check if this trajectory contains both affine and joint DOFs
        cspec = traj.GetConfigurationSpecification()
        needs_base = prpy.util.HasAffineDOFs(cspec)
        needs_joints = prpy.util.HasJointDOFs(cspec)
        if needs_base and needs_joints:
            raise ValueError('Trajectories with affine and joint DOFs are not supported')

        # Check that the current configuration of the robot matches the
        # initial configuration specified by the trajectory.
        if not prpy.util.IsAtTrajectoryStart(self, traj):
            raise TrajectoryNotExecutable(
                'Trajectory started from different configuration than robot.')

        # If there was only one waypoint, at this point we are done!
        if traj.GetNumWaypoints() == 1:
            return traj

        # Verify that the trajectory is timed by checking whether the first
        # waypoint has a valid deltatime value.
        if not prpy.util.IsTimedTrajectory(traj):
            raise ValueError('Trajectory cannot be executed, it is not timed.')

        # Verify that the trajectory has non-zero duration.
        if traj.GetDuration() <= 0.0:
            logger.warning('Executing zero-length trajectory. Please update the'
                          ' function that produced this trajectory to return a'
                          ' single-waypoint trajectory.', FutureWarning)

        traj_manipulators = self.GetTrajectoryManipulators(traj)
        controllers_manip = []
        active_controllers = []
        if self.head in traj_manipulators:
            # TODO head after Schunk integration
            if len(traj_manipulators) == 1:
                raise NotImplementedError('The head is currently disabled under ros_control.')
            else:
                logger.warning('The head is currently disabled under ros_control.')

        # logic to determine which controllers are needed
        if (self.right_arm in traj_manipulators and
                not self.right_arm.IsSimulated() and
                self.left_arm in traj_manipulators and
                not self.left_arm.IsSimulated()):
            controllers_manip.append('bimanual_trajectory_controller')
        else:
            if self.right_arm in traj_manipulators:
                if not self.right_arm.IsSimulated():
                    controllers_manip.append('right_trajectory_controller')
                else:
                    active_controllers.append(self.right_arm.sim_controller)

            if self.left_arm in traj_manipulators:
                if not self.left_arm.IsSimulated():
                    controllers_manip.append('left_trajectory_controller')
                else:
                    active_controllers.append(self.left_arm.sim_controller)

        # load and activate controllers
        if not self.full_controller_sim:
            self.controller_manager.request(controllers_manip).switch()

        # repeat logic and actually construct controller clients
        # now that we've activated them on the robot
        if 'bimanual_trajectory_controller' in controllers_manip:
            joints = []
            joints.extend(self.right_arm.GetJointNames())
            joints.extend(self.left_arm.GetJointNames())
            active_controllers.append(
                RewdOrTrajectoryController(self, '',
                                           'bimanual_trajectory_controller',
                                           joints))
        else:
            if 'right_trajectory_controller' in controllers_manip:
                active_controllers.append(
                    RewdOrTrajectoryController(self, '',
                                               'right_trajectory_controller',
                                               self.right_arm.GetJointNames()))

            if 'left_trajectory_controller' in controllers_manip:
                active_controllers.append(
                    RewdOrTrajectoryController(self, '',
                                               'left_trajectory_controller',
                                               self.left_arm.GetJointNames()))

        if needs_base:
            if (hasattr(self, 'base') and hasattr(self.base, 'controller') and
                    self.base.controller is not None):
                active_controllers.append(self.base.controller)
            else:
                logger.warning(
                    'Trajectory includes the base, but no base controller is'
                    ' available. Is self.base.controller set?')

        for controller in active_controllers:
            controller.SetPath(traj)

        prpy.util.WaitForControllers(active_controllers, timeout=timeout)
        return traj

    def ExecuteTrajectory(self, traj, *args, **kwargs):
        # from prpy.exceptions import TrajectoryAborted

        value = self._ExecuteTrajectory(traj, *args, **kwargs)

        # TODO meaningful to do this check here?
        # if so can be done on value future
        #
        # for manipulator in active_manipulators:
        #     status = manipulator.GetTrajectoryStatus()
        #     if status == 'aborted':
        #         raise TrajectoryAborted()

        return value

    # Inherit docstring from the parent class.
    ExecuteTrajectory.__doc__ = Robot.ExecuteTrajectory.__doc__

    def SetStiffness(self, stiffness, manip=None):
        """Set the stiffness of HERB's arms and head.
        Stiffness False/0 is gravity compensation and stiffness True/(>0) is position
        control.
        @param stiffness boolean or numeric value 0.0 to 1.0
        """
        if (isinstance(stiffness, numbers.Number) and
                not (0 <= stiffness and stiffness <= 1)):
            raise Exception('Stiffness must be boolean or numeric in the range [0, 1];'
                            'got {}.'.format(stiffness))

        # TODO head after Schunk integration
        if manip is self.head:
            raise NotImplementedError('Head immobilized under ros_control, SetStiffness not available.')

        new_manip_controllers = []
        if stiffness:
            if not self.left_arm.IsSimulated() and (manip is None or manip is self.left_arm):
                new_manip_controllers.append('left_joint_group_position_controller')
            if not self.right_arm.IsSimulated() and (manip is None or manip is self.right_arm):
                new_manip_controllers.append('right_joint_group_position_controller')
        else:
            if not self.left_arm.IsSimulated() and (manip is None or manip is self.left_arm):
                new_manip_controllers.append(
                    'left_gravity_compensation_controller')
            if not self.right_arm.IsSimulated() and (manip is None or manip is self.right_arm):
                new_manip_controllers.append(
                    'right_gravity_compensation_controller')

        if not self.full_controller_sim:
            self.controller_manager.request(new_manip_controllers).switch()

    def Say(self, words, block=True):
        """Speak 'words' using talker action service or espeak locally in simulation"""
        if self.talker_simulated:
            try:
                proc = subprocess.Popen(['espeak', '-s', '160', '"{0}"'.format(words)])
                if block:
                    proc.wait()
            except OSError as e:
                logger.error('Unable to speak. Make sure "espeak" is installed locally.\n%s' % str(e))
        else:
            import talker.msg
            goal = talker.msg.SayGoal(text=words)
            self._say_action_client.send_goal(goal)
            if block:
                self._say_action_client.wait_for_result()
