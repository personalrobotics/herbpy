import cbirrt, logging, numpy, openravepy, os, tempfile
import prrave.tsr
import planner

class JacobianPlanner(planner.Planner):
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.module = openravepy.RaveCreateProblem(self.env, 'Manipulation')
        self.env.LoadProblem(self.module, robot.GetName())

    def GetName(self):
        return 'jacobian_planner'

    def PlanToEndEffectorOffset(self, direction, distance):
        with tempfile.NamedTemporaryFile(delete=False) as temp_file:
            traj_path = temp_file.name

        # Plan with JMoveHandStraight and write the trajectory to disk.
        # FIXME: Why doesn't this move hand straight?
        args  = [ 'JMoveHandStraight' ]
        args += [ 'direction' ] + [ str(x) for x in direction ]
        args += [ 'maxdist', str(distance) ]
        args += [ 'execute', '0' ]
        args += [ 'writetraj', traj_path ]
        args_str = ' '.join(args)
        response = self.module.SendCommand(args_str)

        # Load and return the trajectory.
        # TODO: Delete the temporary trajectory file.
        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()

        try:
            # TODO: Load trajectories with the prrave helper function.
            traj = openravepy.RaveCreateTrajectory(self.env, '')
            traj.deserialize(traj_xml)
        except openravepy.openrave_exception, e:
            raise planning.PlanningError('Planning with JMoveHandStraight failed.')

        # TODO: Verify that the trajectory moved the full distance.
        return traj
