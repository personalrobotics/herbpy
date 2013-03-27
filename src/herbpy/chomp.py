import openravepy, orcdchomp.orcdchomp

class CHOMPPlanner:
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.module = openravepy.RaveCreateModule(self.env, 'orcdchomp')
        orcdchomp.orcdchomp.bind(self.module)

    def PlanToConfiguration(self, goal, **kw_args):
        # Compute the distance field.
        for body in self.env.GetBodies():
            self.module.computedistancefield(body)

        # Plan with CHOMP.
        return self.module.runchomp(robot=self.robot, adofgoal=goal, lambda_=100.0, n_iter=100,
                                    no_collision_exception=True)

    def PlanToEndEffectorPose(self, goal_pose, psample=0.1, **kw_args):
        return False
