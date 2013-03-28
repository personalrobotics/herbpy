import logging, rospkg, os
import openravepy, orcdchomp.orcdchomp
from planner import Planner, PlanningError

class CHOMPPlanner(Planner):
    def __init__(self, robot):
        self.initialized = False
        self.env = robot.GetEnv()
        self.robot = robot
        self.module = openravepy.RaveCreateModule(self.env, 'orcdchomp')
        orcdchomp.orcdchomp.bind(self.module)

    def GetName(self):
        return 'chomp'

    def PlanToConfiguration(self, goal, **kw_args):
        if not self.initialized:
            logging.warning('CHOMP requires a distance field to be loaded.')
            raise NotImplementedError

        with self.robot.CreateRobotStateSaver(self.robot):
            try:
                return self.module.runchomp(robot=self.robot, adofgoal=goal, **kw_args)
            except RuntimeError, e:
                logging.warning('CHOMP returned error {0:s}'.format(e))
                raise PlanningError

    def ComputeDistanceField(self):
        with self.env:
            # Compute the distance field for the non-spherized parts of HERB. This
            # includes everything that isn't attached to an arm. Otherwise the
            # initial arm will be incorrectly added to the distance field.
            with self.robot.CreateRobotStateSaver():
                logging.info("Creating the robot's distance field.")
                proximal_joints = [ manip.GetArmIndices()[0] for manip in self.robot.GetManipulators() ]
                for link in self.robot.GetLinks():
                    for proximal_joint in proximal_joints:
                        if self.robot.DoesAffect(proximal_joint, link.GetIndex()):
                            link.Enable(False)

                cache_path = self.GetCachePath(self.robot)
                self.module.computedistancefield(self.robot, cache_filename=cache_path)

            # Compute a separate distance field every other object.
            savers = list()
            for body in self.env.GetBodies():
                savers.append(body.CreateKinBodyStateSaver())
                body.Enable(False)

            for body in self.env.GetBodies():
                if body != self.robot:
                    logging.info("Creating distance field for '{0:s}'.".format(body.GetName()))
                    body.Enable(True)
                    cache_path = self.GetCachePath(body)
                    self.module.computedistancefield(body, cache_filename=cache_path)
                    body.Enable(False)

        self.initialized = True 
        del savers

    def GetCachePath(self, body):
        cache_dir = rospkg.get_ros_home()
        cache_name = '{0:s}.chomp'.format(self.robot.GetKinematicsGeometryHash())
        return os.path.join(cache_dir, cache_name)
