import cbirrt, logging, numpy, openravepy, os, tempfile
import prrave.tsr
import planner

class CBiRRTPlanner(planner.Planner):
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.problem = openravepy.RaveCreateProblem(self.env, 'CBiRRT')
        self.env.LoadProblem(self.problem, robot.GetName())

    def GetName(self):
        return 'cbirrt'

    def Plan(self, smoothingitrs=None, timelimit=None, allowlimadj=None, extra_args=None, **kw_args):
        args = [ 'RunCBiRRT' ]
        if extra_args is not None:
            args += extra_args
        if smoothingitrs is not None:
            args += [ 'smoothingitrs', smoothingitrs ]
        if timelimit is not None:
            args += [ 'timelimit', timelimit ]
        if allowlimadj is not None and allowlimadj:
            args += [ 'allowlimadj', '1' ]

        # FIXME: Why can't we write to anything other than cmovetraj.txt or
        # /tmp/cmovetraj.txt with CBiRRT?
        traj_path = '/tmp/cmovetraj.txt'
        args += [ 'filename', traj_path ]
        args_str = ' '.join(args)
        response = self.problem.SendCommand(args_str)
        if int(response) != 1:
            raise planner.PlanningError('Planning with CBiRRT failed.')

        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()
            traj = openravepy.RaveCreateTrajectory(self.env, '')
            traj.deserialize(traj_xml)

        return traj

    def PlanToConfiguration(self, goal, **kw_args):
        goal_array = numpy.array(goal)
        if len(goal_array) != self.robot.GetActiveDOF():
            logging.error('Incorrect number of DOFs in goal configuration; expected {0:d}, got {1:d}'.format(
                          self.robot.GetActiveDOF(), len(goal_array)))
            raise planner.PlanningError('Incorrect number of DOFs in goal configuration.')

        extra_args = [ 'jointgoals', str(len(goal_array)), ' '.join([ str(x) for x in goal_array ]) ]
        return self.Plan(extra_args=extra_args, **kw_args)

    def PlanToEndEffectorPose(self, goal_pose, psample=0.1, **kw_args):
        manipulator_index = self.robot.GetActiveManipulatorIndex()
        goal_tsr = prrave.tsr.TSR(T0_w=goal_pose, manip=manipulator_index)
        tsr_chain = prrave.tsr.TSRChain(sample_goal=True, TSR=goal_tsr)

        extra_args  = [ 'TSRChain', tsr_chain.serialize() ]
        extra_args += [ 'psample', str(psample) ]
        return self.Plan(extra_args=extra_args, **kw_args)
