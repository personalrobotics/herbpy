import openravepy, orcdchomp.orcdchomp
import inspect
from methodlist_decorator import CreateMethodListDecorator

PlanningMethod = CreateMethodListDecorator()

class PlanningError(openravepy.openrave_exception):
    def __init__(self, message):
        super(openrave.openrave_exception, self).__init__(message)

class UnsupportedPlanningError(PlanningError):
    def __init__(self, message):
        super(PlanningError, self).__init__(message)

class Planner:
    def GetName(self):
        return 'unknown planner'

    @PlanningMethod
    def PlanToConfiguration(self, goal, **kw_args):
        raise UnsupportedPlanningError('This planner does not support PlanToConfiguration.')

    @PlanningMethod
    def PlanToEndEffectorPose(self, goal_pose):
        raise UnsupportedPlanningError('This planner does not support PlanToEndEffectorPose.')

    @PlanningMethod
    def PlanToEndEffectorOffset(self, direction, distance):
        raise UnsupportedPlanningError('This planner does not support PlanToEndEffectorOffset.')
