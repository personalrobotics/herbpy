import logging, planner, openravepy

class SnapPlanner:
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot

    def GetName(self):
        return 'snap'

    def PlanToConfiguration(self, goal, snap_tolerance=0.1, **kw_args):
        active_indices = self.robot.GetActiveDOFIndices()
        current_dof_values = self.robot.GetActiveDOFValues()

        # Only snap if we're close to the goal configuration.
        if (goal - current_dof_values).max() > snap_tolerance:
            raise planner.UnsupportedPlanningError

        # Create a two-point trajectory that takes us to the goal.
        logging.info('Snapping to goal configuration with a straight line trajectory.')
        traj = openravepy.RaveCreateTrajectory(self.env, '')
        config_spec = self.robot.GetActiveConfigurationSpecification()
        traj.Init(config_spec)
        traj.Insert(0, current_dof_values, config_spec)
        traj.Insert(1, goal, config_spec)
        openravepy.planningutils.RetimeTrajectory(traj)
        return traj
