import collections, functools, inspect, numpy, openravepy, types, time, logging, termcolor
import herbpy, prrave.rave

class ColoredFormatter(logging.Formatter):
    def __init__(self, default):
        self._default_formatter = default
        self._color_table = collections.defaultdict(lambda: list())
        self._color_table[logging.CRITICAL] = [ 'red' ]
        self._color_table[logging.ERROR] = [ 'red' ]
        self._color_table[logging.WARNING] = [ 'yellow' ]
        self._color_table[logging.DEBUG] = [ 'green' ]

    def format(self, record):
        color_options = self._color_table[record.levelno]
        message = self._default_formatter.format(record)
        return termcolor.colored(message, *color_options)

class Deprecated(object):
    def __init__(self, message):
        self._message = message

    # TODO: Set the help text and name to the wrapped function.
    def __call__(self, f):
        @functools.wraps(f)
        def wrapped_function(*args, **kw_args):
            if self._message is None:
                logging.warning('%s is deprecated.', f.__name__)
            else:
                logging.warning('%s is deprecated: %s', f.__name__, self._message)

            return f(*args, **kw_args)
        return wrapped_function

def CreateMethodListDecorator():
    class MethodListDecorator(object):
        methods = list()

        def __init__(self, func):
            self._func = func
            self.__class__.methods.append(func)

        def __get__(self, obj, type=None):
            return self.__class__(self._func.__get__(obj, type))

        def __call__(self, *args, **kw_args):
            return self._func(*args, **kw_args)

        @classmethod
        def Bind(cls, instance):
            for method in cls.methods:
                if inspect.isfunction(method):
                    bound_method = types.MethodType(method, instance, type(instance))
                else:
                    print method
                    bound_method = method
                setattr(instance, method.__name__, bound_method)

    return MethodListDecorator

def intercept_bind(cls, callback):
    def intercept(self, name):
        try:
            object.__getattribute__(self, '_intercepted')
        except AttributeError:
            self._intercepted = True
            callback(self)

        return object.__getattribute__(self, name)

    cls.__getattribute__ = intercept

def ExtractWorkspaceWaypoints(robot, traj):
    # Extract the DOF indices from the trajectory.
    config_spec = traj.GetConfigurationSpecification()
    group = config_spec.GetGroupFromName('joint_values')
    dof_indices = [ int(index) for index in group.name.split()[2:] ]

    with robot.GetEnv():
        with robot.CreateRobotStateSaver():
            manip = robot.GetActiveManipulator()
            robot.SetActiveDOFs(dof_indices)

            # Compute the hand pose at each waypoint using forward kinematics.
            waypoints = list()
            for t in xrange(traj.GetNumWaypoints()):
                waypoint = traj.GetWaypoint(t)
                q = config_spec.ExtractJointValues(waypoint, robot, dof_indices)
                robot.SetActiveDOFValues(q)
                hand_pose = manip.GetEndEffectorTransform()
                waypoints.append(hand_pose)

    return waypoints

def GetTrajectoryIndices(traj):
    joint_values_group = traj.GetConfigurationSpecification().GetGroupFromName('joint_values')
    return [ int(index) for index in joint_values_group.name.split()[2:] ]

def GetTrajectoryManipulators(robot, traj):
    traj_indices = set(GetTrajectoryIndices(traj))

    active_manipulators = []
    for manipulator in robot.manipulators:
        manipulator_indices = set(manipulator.GetArmIndices())
        if traj_indices & manipulator_indices:
            active_manipulators.append(manipulator)

    return active_manipulators

class Timer:
    def __init__(self, message):
        self.message = message
        self.start = 0 

    def __enter__(self):
        logging.info('%s started execution.', self.message)
        self.start = time.time()

    def __exit__(self, exc_type, exc_value, traceback):
        self.end = time.time()
        self.duration = self.end - self.start
        logging.info('%s executed in %.5f seconds.', self.message, self.duration)

class RenderTrajectory:
    """
    Context manager for rendering trajectories in the OpenRAVE viewer. This
    renders a trajectory as a smooth curve. The curve is are removed when the exit
    handler is called.
    @param robot robot executing the trajectory
    @param traj input trajectory
    @param num_samples number of samples to use for interpolation
    @param radius sphere radius for waypoint indicators
    @param color interpolated line color
    @param interpolation flag to enable or disable path rendering
    @param waypoints flag to enable or disable waypoint rendering
    """
    def __init__(self, robot, traj, num_samples=20, linewidth=2, color=[1, 0, 0, 1]):
        self.env = robot.GetEnv()
        self.robot = robot
        self.handles = list()

        # Rendering options.
        self.num_samples = num_samples
        self.linewidth = linewidth
        self.color = numpy.array(color, dtype='float')

        # Clone and retime the trajectory.
        self.traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')
        self.traj.Clone(traj, 0)
        openravepy.planningutils.RetimeTrajectory(self.traj)

    def __enter__(self):
        with self.env:
            with self.robot.CreateRobotStateSaver():
                config_spec = self.traj.GetConfigurationSpecification()
                manipulators = GetTrajectoryManipulators(self.robot, self.traj)

                for manipulator in manipulators:
                    arm_indices = manipulator.GetArmIndices()

                    # Skip manipulators that don't have render_offset set.
                    render_offset = manipulator.render_offset
                    if render_offset is None:
                        continue

                    # Evenly interpolate joint values throughout the entire trajectory.
                    interpolated_points = list()
                    for t in numpy.linspace(0, self.traj.GetDuration(), self.num_samples):
                        waypoint = self.traj.Sample(t)
                        joint_values = config_spec.ExtractJointValues(waypoint, self.robot, arm_indices)
                        manipulator.SetDOFValues(joint_values)
                        hand_pose = manipulator.GetEndEffectorTransform()
                        render_position = numpy.dot(hand_pose, render_offset)
                        interpolated_points.append(render_position[0:3])

                    # Render a line through the interpolated points.
                    interpolated_points = numpy.array(interpolated_points)
                    if len(interpolated_points) > 0:
                        handle = self.env.drawlinestrip(interpolated_points, self.linewidth, self.color)
                        self.handles.append(handle)

    def __exit__(self, *exc_info):
        with self.env:
            del self.handles
