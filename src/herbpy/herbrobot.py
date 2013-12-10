PACKAGE = 'herbpy'
import logging, numpy, openravepy, rospy, time
import prpy

logger = logging.getLogger('herbpy')

# Absolute path to this package.
from rospkg import RosPack
ros_pack = RosPack()
PACKAGE_PATH = ros_pack.get_path(PACKAGE)

class HERBRobot(prpy.base.WAMRobot):
    def __init__(self, left_arm_sim, right_arm_sim, right_ft_sim,
                       left_hand_sim, right_hand_sim, left_ft_sim,
                       head_sim, moped_sim, talker_sim, segway_sim):
        prpy.base.WAMRobot.__init__(self)

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
        from prpy.base import BarrettHand, WAM
        from herbpantilt import HERBPantilt
        prpy.bind_subclass(self.left_arm, WAM, sim=left_arm_sim, owd_namespace='/left/owd')
        prpy.bind_subclass(self.right_arm, WAM, sim=right_arm_sim, owd_namespace='/right/owd')
        prpy.bind_subclass(self.head, HERBPantilt, sim=head_sim, owd_namespace='/head/owd')
        prpy.bind_subclass(self.left_arm.hand, BarrettHand, sim=left_hand_sim, manipulator=self.left_arm,
                           owd_namespace='/left/owd', bhd_namespace='/left/bhd', ft_sim=right_ft_sim)
        prpy.bind_subclass(self.right_arm.hand, BarrettHand, sim=right_hand_sim, manipulator=self.right_arm,
                           owd_namespace='/right/owd', bhd_namespace='/right/bhd', ft_sim=right_ft_sim)

        # Setup the segway controller
        self.segway_controller = self.AttachController(name=self.GetName(),
                                                       args='SegwayController {0:s}'.format('herbpy'),
                                                       dof_indices=[],
                                                       affine_dofs=openravepy.DOFAffine.Transform,
                                                       simulated=segway_sim)

        
        # Support for named configurations.
        import os.path
        self.configurations.add_group('left_arm', self.left_arm.GetArmIndices())
        self.configurations.add_group('right_arm', self.right_arm.GetArmIndices())
        self.configurations.add_group('head', self.head.GetArmIndices())
        self.configurations.add_group('left_hand', self.left_hand.GetIndices())
        self.configurations.add_group('right_hand', self.right_hand.GetIndices())
        try:
            configurations_path = os.path.join(PACKAGE_PATH, 'config/configurations.yaml')
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            logger.warning('Failed loading named configurations from %s.', configurations_path)

        # Initialize a default planning pipeline.
        from prpy.planning import Planner, Sequence, Ranked, Fastest
        from prpy.planning import CBiRRTPlanner, CHOMPPlanner, IKPlanner, MKPlanner, NamedPlanner, SnapPlanner
        self.cbirrt_planner = CBiRRTPlanner()
        self.chomp_planner = CHOMPPlanner()
        self.mk_planner = MKPlanner()
        self.snap_planner = SnapPlanner()
        self.named_planner = NamedPlanner()
        self.ik_planner = IKPlanner()
        self.planner = Sequence(self.ik_planner,
                                self.named_planner,
                                self.snap_planner, 
                                Ranked(self.chomp_planner,
                                       Sequence(self.mk_planner,
                                                self.cbirrt_planner)))

        # Setting necessary sim flags
        self.talker_simulated = talker_sim
        self.segway_sim = segway_sim
        self.moped_sim = moped_sim
        if not self.moped_sim:
          args = 'MOPEDSensorSystem herbpy /moped/ map'
          self.moped_sensorsystem = openravepy.RaveCreateSensorSystem(self.GetEnv(), args)
          if self.moped_sensorsystem is None:
            raise Exception("creating the MOPED sensorsystem failed")

    def CloneBindings(self, parent):
        from prpy import Cloned
        prpy.base.WAMRobot.CloneBindings(self, parent)
        self.left_arm = Cloned(parent.left_arm)
        self.right_arm = Cloned(parent.right_arm)
        self.head = Cloned(parent.head)
        self.left_arm.hand = Cloned(parent.left_arm.GetEndEffector())
        self.right_arm.hand = Cloned(parent.right_arm.GetEndEffector())
        self.left_hand = self.left_arm.hand
        self.right_hand = self.right_arm.hand
        self.manipulators = [ self.left_arm, self.right_arm, self.head ]
        self.planner = parent.planner

    def Say(robot, message):
        """
        Say a message using HERB's text-to-speech engine.
        @param message
        """
        from pr_msgs.srv import AppletCommand

        if not robot.talker_simulated:
            # XXX: HerbPy should not make direct service calls.
            logger.info('Saying "%s".', message)
            #rospy.wait_for_service('/talkerapplet')
            talk = rospy.ServiceProxy('/talkerapplet', AppletCommand)    
            try:
                talk('say', message, 0, 0)
            except rospy.ServiceException, e:
                logger.error('Error talking.')

    def WaitForObject(robot, obj_name, timeout=None, update_period=0.1):
        start = time.time()
        found_body = None

        if not robot.moped_sim:
            robot.moped_sensorsystem.SendCommand('Enable')
        else:
            # Timeout immediately in simulation.
            timeout = 0

        logger.info("Waiting for object %s to appear.", obj_name)
        try:
            while True:
                # Check for an object with the appropriate name in the environment.
                bodies = robot.GetEnv().GetBodies()
                for body in bodies:
                    if body.GetName().startswith('moped_' + obj_name):
                        return body

                # Check for a timeout.
                if timeout is not None and time.time() - start >= timeout:
                    logger.info("Timed out without finding object.")
                    return None

                time.sleep(update_period)
        finally:
            if not robot.moped_sim:
                robot.moped_sensorsystem.SendCommand('Disable')

    def DriveStraightUntilForce(robot, direction, velocity=0.1, force_threshold=3.0,
                                max_distance=None, timeout=None, left_arm=True, right_arm=True):
        """
        Drive the base in a direction until a force/torque sensor feels a force. The
        Segway first turns to face the desired direction, then drives forward at the
        specified velocity. The action terminates when max_distance is reached, the
        timeout is exceeded, or if a force is felt. The maximum distance and timeout
        can be disabled by setting the corresponding parameters to None.
        @param direction forward direction of motion in the world frame
        @param velocity desired forward velocity
        @param force_threshold threshold force in Newtons
        @param max_distance maximum distance in meters
        @param timeout maximum duration in seconds
        @param left_arm flag to use the left force/torque sensor
        @param right_arm flag to use the right force/torque sensor
        @return felt_force flag indicating whether the action felt a force
        """
        if robot.segway_sim:
            logger.warning('DriveStraightUntilForce does not work with a simulated Segway.')
            return

        if (robot.left_ft_sim and left_arm) or (robot.right_ft_sim and right_arm):
            raise Exception('DriveStraightUntilForce does not work with simulated force/torque sensors.')

        with prpy.util.Timer("Drive segway until force"):
            env = robot.GetEnv()
            direction = numpy.array(direction, dtype='float')
            direction /= numpy.linalg.norm(direction) 
            manipulators = list()
            if left_arm:
                manipulators.append(robot.left_arm)
            if right_arm:
                manipulators.append(robot.right_arm)

            if not manipulators:
                logger.warning('Executing DriveStraightUntilForce with no force/torque sensor for feedback.')

            # Rotate to face the right direction.
            with env:
                robot_pose = robot.GetTransform()
            robot_angle = numpy.arctan2(robot_pose[1, 0], robot_pose[0, 0])
            desired_angle = numpy.arctan2(direction[1], direction[0])
            robot.RotateSegway(desired_angle - robot_angle)


            # Soft-tare the force/torque sensors. Tare is too slow.
            initial_force = dict()
            for manipulator in manipulators:
                force, torque = manipulator.hand.GetForceTorque()
                initial_force[manipulator] = force
            
            try:
                felt_force = False
                start_time = time.time()
                start_pos = robot_pose[0:3, 3]
                while True:
                    # Check if we felt a force on any of the force/torque sensors.
                    for manipulator in manipulators:
                        force, torque = manipulator.hand.GetForceTorque()
                        if numpy.linalg.norm(initial_force[manipulator] - force) > force_threshold:
                            return True

                    # Check if we've exceeded the maximum distance.
                    with env:
                        current_pos = robot.GetTransform()[0:3, 3]
                    distance = numpy.dot(current_pos - start_pos, direction)
                    if max_distance is not None and distance >= max_distance:
                        return False

                    # Check for a timeout.
                    time_now = time.time()
                    if timeout is not None and time_now - star_time > timeout:
                        return False

                    # Continuously stream forward velocities.
                    robot.segway_controller.SendCommand('DriveInstantaneous {0:f} 0 0'.format(velocity))
            finally:
                # Stop the Segway before returning.
                robot.segway_controller.SendCommand('DriveInstantaneous 0 0 0')

    def DriveAlongVector(robot, direction, goal_pos):
        direction = direction[:2]/numpy.linalg.norm(direction[:2])
        herb_pose = robot.GetTransform()
        distance = numpy.dot(goal_pos[:2]-herb_pose[:2,3], direction)
        cur_angle = numpy.arctan2(herb_pose[1,0],herb_pose[0,0])
        des_angle = numpy.arctan2(direction[1],direction[0])
        robot.RotateSegway(des_angle-cur_angle)
        robot.DriveSegway(distance)

    def DriveSegway(robot, meters, timeout=None):
        with prpy.util.Timer("Drive segway"):
            if not robot.segway_sim:
                robot.segway_controller.SendCommand("Drive " + str(meters))
                running_controllers = [ robot.segway_controller ]
                is_done = prpy.util.WaitForControllers(running_controllers, timeout=timeout)
            # Create and execute base trajectory in simulation.
            else:
                with robot.GetEnv():
                    current_pose = robot.GetTransform().copy()
                    current_pose[0:3,3] = current_pose[0:3,3] + meters*current_pose[0:3,0]
                    robot.SetTransform(current_pose)

    def DriveSegwayToNamedPosition(robot, named_position):
        if robot.segway_sim:
            raise Exception('Driving to named positions is not supported in simulation.')
        else:
            robot.segway_controller.SendCommand("Goto " + named_position)

    def RotateSegway(robot, angle_rad, timeout=None):
        with prpy.util.Timer("Rotate segway"):
            if robot.segway_sim:
                with robot.GetEnv():
                    current_pose_in_world = robot.GetTransform().copy()
                    desired_pose_in_herb = numpy.array([[numpy.cos(angle_rad), -numpy.sin(angle_rad), 0, 0],
                                                        [numpy.sin(angle_rad), numpy.cos(angle_rad), 0, 0],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]])
                    desired_pose_in_world = numpy.dot(current_pose_in_world, desired_pose_in_herb)
                    robot.SetTransform(desired_pose_in_world)
            else:
                robot.segway_controller.SendCommand("Rotate " + str(angle_rad))
                running_controllers = [ robot.segway_controller ]
                is_done = prpy.util.WaitForControllers(running_controllers, timeout=timeout)

    def StopSegway(robot):
        if not robot.segway_sim:
            robot.segway_controller.SendCommand("Stop")
