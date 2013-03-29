import openravepy, orcdchomp.orcdchomp

class PlanningError(openravepy.openrave_exception):
    def __init__(self, message):
        super(openrave.openrave_exception, self).__init__(message)

class UnsupportedPlanningError(PlanningError):
    def __init__(self, message):
        super(PlanningError, self).__init__(message)

class Planner:
    def GetName(self):
        return 'unknown planner'

    def PlanToConfiguration(self, goal, **kw_args):
        raise UnsupportedPlanningError('This planner does not support PlanToConfiguration.')

    def PlanToEndEffectorPose(self, goal_pose):
        raise UnsupportedPlanningError('This planner does not support PlanToEndEffectorPose.')

