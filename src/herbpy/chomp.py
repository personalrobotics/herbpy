import logging
import openravepy, orcdchomp.orcdchomp
from planner import Planner, PlanningError

class CHOMPPlanner(Planner):
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.module = openravepy.RaveCreateModule(self.env, 'orcdchomp')
        orcdchomp.orcdchomp.bind(self.module)

    def GetName(self):
        return 'chomp'

    def PlanToConfiguration(self, goal, **kw_args):
        self.ComputeDistanceField()
        try:
            return self.module.runchomp(robot=self.robot, adofgoal=goal, **kw_args)
        except RuntimeError, e:
            logging.warning('CHOMP returned error {0:s}'.format(e))
            raise PlanningError

    def ComputeDistanceField(self):
        for body in self.env.GetBodies():
            self.module.computedistancefield(body)
