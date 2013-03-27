import openravepy, orcdchomp.orcdchomp

class Planner:
    def PlanToConfiguration(self, goal, **kw_args):
        raise NotImplementedError

    def PlanToEndEffectorPose(self, goal_pose):
        raise NotImplementedError

class PlanningError:
    def __init__(self):
        pass
