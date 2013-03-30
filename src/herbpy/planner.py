import openravepy, orcdchomp.orcdchomp
import inspect
import util

PlanningMethod = util.CreateMethodListDecorator()

class PlanningError(Exception):
    pass

class UnsupportedPlanningError(PlanningError):
    pass

class Planner:
    def GetName(self):
        """
        Gets the planner's name.
        @return planner_name
        """
        return 'unknown planner'

    @PlanningMethod
    def PlanToConfiguration(self, goal, **kw_args):
        """
        Plan from the robot's current configuration to a goal configuration.
        @param goal desired configuration
        @return trajectory
        """
        raise UnsupportedPlanningError('This planner does not support PlanToConfiguration.')

    @PlanningMethod
    def PlanToEndEffectorPose(self, goal_pose, **kw_args):
        """
        Plan from the robot's current configuration to a goal end-effector pose.
        @param goal_pose desired end-effector pose
        @return trajectory
        """
        raise UnsupportedPlanningError('This planner does not support PlanToEndEffectorPose.')

    @PlanningMethod
    def PlanToEndEffectorOffset(self, direction, distance, **kw_args):
        """
        Plan a straight motion in the world frame.
        @param direction unit vector in the direction of motion
        @param distance distance along the direction of motion
        @return trajectory
        """
        raise UnsupportedPlanningError('This planner does not support PlanToEndEffectorOffset.')
