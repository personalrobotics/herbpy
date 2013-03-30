import logging, rospkg, os
import openravepy, orcdchomp.orcdchomp
import planner

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

        with self.robot.CreateRobotStateSaver(self.robot):
            try:
                return self.module.runchomp(robot=self.robot, adofgoal=goal,
                                            lambda_=lambda_, n_iter=n_iter, **kw_args)
            except RuntimeError, e:
                raise planner.PlanningError(str(e))

    def ComputeDistanceField(self):
        with self.env:
            # Disable everything.
            savers = list()
            for body in self.env.GetBodies():
                saver = body.CreateKinBodyStateSaver()
                saver.__enter__()
                savers.append(saver)
                body.Enable(False)

            # Compute the distance field for the non-spherized parts of HERB. This
            # includes everything that isn't attached to an arm. Otherwise the
            # initial arm will be incorrectly added to the distance field.
            self.robot.Enable(True)
            logging.info("Creating the robot's distance field.")
            proximal_joints = [ manip.GetArmIndices()[0] for manip in self.robot.GetManipulators() ]
            for link in self.robot.GetLinks():
                for proximal_joint in proximal_joints:
                    if not self.robot.DoesAffect(proximal_joint, link.GetIndex()):
                        link.Enable(True)

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
        cache_name = '{0:s}.chomp'.format(self.robot.GetKinematicsGeometryHash())
        return os.path.join(cache_dir, cache_name)
