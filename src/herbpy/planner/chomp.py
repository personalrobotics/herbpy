import contextlib, logging, rospkg, os
import openravepy, numpy, orcdchomp.orcdchomp
import prrave.tsr
import planner
import tempfile
import herbpy

class CHOMPPlanner(planner.Planner):
    def __init__(self, robot):
        self.initialized = False
        self.env = robot.GetEnv()
        self.robot = robot
        self.module = openravepy.RaveCreateModule(self.env, 'orcdchomp')
        orcdchomp.orcdchomp.bind(self.module)

    def GetName(self):
        return 'chomp'

    def PlanToConfiguration(self, goal, lambda_=100.0, n_iter=100, **kw_args):
        if not self.initialized:
            raise planner.UnsupportedPlanningError('CHOMP requires a distance field.')

        try:
            with self.robot.CreateRobotStateSaver(self.robot):
                return self.module.runchomp(robot=self.robot, adofgoal=goal,
                                            lambda_=lambda_, n_iter=n_iter, **kw_args)
        except RuntimeError, e:
            try:
                with tempfile.NamedTemporaryFile(delete=False) as log_file:
                    # Serialize kinbodies.
                    for body in self.env.GetBodies():
                        log_file.write('KINBODY name = "{name:s}", transform = {pose:s}, enabled = {enabled:d}\n'.format(
                            name = body.GetName(),
                            pose = repr(body.GetTransform()),
                            enabled = body.IsEnabled()
                        ))

                    # Start and goal configurations.
                    log_file.write('STARTCONFIGURATION {0:s}\n'.format(repr(self.robot.GetDOFValues())))
                    log_file.write('GOALCONFIGURATION {0:s}\n'.format(repr(goal)))

                    # CHOMP parameters.
                    log_file.write('LAMBDA {0:f} NITER {1:f}\n'.format(lambda_, n_iter))
                    log_file.write('OTHERARGS {0:s}\n'.format(repr(kw_args)))
                    log_file.write('ERROR {0:s}\n'.format(str(e)))

                herbpy.logger.error('Saved CHOMP error log to %s', log_file.name)
            except:
                herbpy.logger.error('Saving CHOMP error log failed.')

            raise planner.PlanningError(str(e))

    def PlanToEndEffectorPose(self, goal_pose, lambda_=100.0, n_iter=100, goal_tolerance=0.01, **kw_args):
        # CHOMP only supports start sets. Instead, we plan backwards from the
        # goal TSR to the starting configuration. Afterwards, we reverse the
        # trajectory.
        # TODO: Replace this with a proper goalset CHOMP implementation.
        manipulator_index = self.robot.GetActiveManipulatorIndex()
        goal_tsr = prrave.tsr.TSR(T0_w=goal_pose, manip=manipulator_index)
        start_config = self.robot.GetActiveDOFValues()

        if not self.initialized:
            raise planner.UnsupportedPlanningError('CHOMP requires a distance field.')

        with self.robot.CreateRobotStateSaver():
            try:
                traj = self.module.runchomp(robot=self.robot, adofgoal=start_config, start_tsr=goal_tsr)
                traj = openravepy.planningutils.ReverseTrajectory(traj)
            except RuntimeError, e:
                raise planner.PlanningError(str(e))

            # Verify that CHOMP didn't converge to the wrong goal. This is a
            # workaround for a bug in GSCHOMP where the constraint projection
            # fails because of joint limits.
            config_spec = traj.GetConfigurationSpecification()
            last_waypoint = traj.GetWaypoint(traj.GetNumWaypoints() - 1)
            final_config = config_spec.ExtractJointValues(last_waypoint, self.robot, self.robot.GetActiveDOFIndices())
            self.robot.SetActiveDOFValues(final_config)
            final_pose = self.robot.GetActiveManipulator().GetEndEffectorTransform()

            # TODO: Also check the orientation.
            goal_distance = numpy.linalg.norm(final_pose[0:3, 3] - goal_pose[0:3, 3])
            if goal_distance > goal_tolerance:
                raise planner.PlanningError('CHOMP deviated from the goal pose by {0:f} meters.'.format(goal_distance))

            return traj

    def ComputeDistanceField(self):
        with self.env:
            # Save the state so we can restore it later.
            savers = [ body.CreateKinBodyStateSaver() for body in self.env.GetBodies() ]

            with contextlib.nested(*savers):
                # Disable everything.
                for body in self.env.GetBodies():
                    body.Enable(False)

                # Compute the distance field for the non-spherized parts of HERB. This
                # includes everything that isn't attached to an arm. Otherwise the
                # initial arm will be incorrectly added to the distance field.
                self.robot.Enable(True)
                logging.info("Creating the robot's distance field.")
                proximal_joints = [ manip.GetArmIndices()[0] for manip in self.robot.GetManipulators() ]
                for link in self.robot.GetLinks():
                    for proximal_joint in proximal_joints:
                        if self.robot.DoesAffect(proximal_joint, link.GetIndex()):
                            link.Enable(False)

                cache_path = self.GetCachePath(self.robot)
                self.module.computedistancefield(self.robot, cache_filename=cache_path)
                self.robot.Enable(False)

                # Compute separate distance fields for all other objects.
                for body in self.env.GetBodies():
                    if body != self.robot:
                        logging.info("Creating distance field for '{0:s}'.".format(body.GetName()))
                        body.Enable(True)
                        cache_path = self.GetCachePath(body)
                        self.module.computedistancefield(body, cache_filename=cache_path)
                        body.Enable(False)

        self.initialized = True 
        for saver in savers:
            saver.Restore()
            saver.Release()

    def GetCachePath(self, body):
        cache_dir = rospkg.get_ros_home()
        cache_name = '{0:s}.chomp'.format(body.GetKinematicsGeometryHash())
        return os.path.join(cache_dir, cache_name)
