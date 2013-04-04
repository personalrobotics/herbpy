import numpy, openravepy, types, time
import herbpy, prrave.rave

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
                bound_method = types.MethodType(method, instance, type(instance))
                setattr(instance, method.__name__, bound_method)

    return MethodListDecorator

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

class Timer:
    def __init__(self, message):
        self.message = message
        self.start = 0 

    def __enter__(self):
        self.start = time.time()

    def __exit__(self, exc_type, exc_value, traceback):
        self.end = time.time()
        self.duration = self.end - self.start
        herbpy.logger.info('%s executed in %.5f seconds.', self.message, self.duration)

class RenderTrajectory:
    """
    Context manager for rendering trajectories in the OpenRAVE viewer. This
    class supports rendering the workspace waypoints as spheres and the entire
    trajectory as a smooth curve. The created objects are removed when the exit
    handler is called. This class assumes that the trajectory is being executed on
    the active manipulator.
    @param robot robot executing the trajectory with its active manipulator
    @param traj trajectory
    @param resolution approximate path discretization in meters
    @param linewidth width of the line passed to the draw function
    @param radius sphere radius for waypoint indicators
    @param color interpolated line color
    @param interpolation flag to enable or disable path rendering
    @param waypoints flag to enable or disable waypoint rendering
    """
    def __init__(self, robot, traj, resolution=0.05, linewidth=2, radius=0.01,
                 color=None, interpolation=True, waypoints=False):
        self.env = robot.GetEnv()
        self.robot = robot

        # Rendering options
        self.resolution = resolution
        self.linewidth = linewidth
        self.sphere_radius = radius
        self.render_interpolation = interpolation
        self.render_waypoints = waypoints

        if color is not None:
            self.color = numpy.array(color)
        else:
            self.color = numpy.array([ 1, 0, 0, 1 ], dtype='float')

        # Handles to the created objects.
        self.lines_handle = None
        self.sphere_handle = None

        # Clone and retime the trajectory.
        self.traj = openravepy.RaveCreateTrajectory(self.env, '')
        self.traj.Clone(traj, 0)
        openravepy.planningutils.RetimeTrajectory(self.traj)

    def __enter__(self):
        # TODO: Properly handle bimanual trajectories.
        config_spec = self.traj.GetConfigurationSpecification()
        group = config_spec.GetGroupFromName('joint_values')
        manip = self.robot.GetActiveManipulator()
        dof_indices = [ int(index) for index in group.name.split()[2:] ]

        with self.env:
            waypoint_poses = ExtractWorkspaceWaypoints(self.robot, self.traj)

            if self.render_waypoints:
                sphere_params = list()
                for waypoint_pose in waypoint_poses:
                    sphere_params.append(numpy.hstack((waypoint_pose[0:3, 3], self.sphere_radius)))

                # Render the trajectory waypoints as spheres.
                self.sphere_handle = openravepy.RaveCreateKinBody(self.env, '')
                self.sphere_handle.InitFromSpheres(numpy.array(sphere_params), True)
                self.sphere_handle.SetName('trajectory')
                self.sphere_handle.Enable(False)
                self.env.Add(self.sphere_handle, True)

            if self.render_interpolation:
                # Estimate the arclength of the trajectory by connecting waypoints
                # with piecewise linear segments in the workspace.
                arclength = 0
                for waypoint1_pose, waypoint2_pose in zip(waypoint_poses[0:], waypoint_poses[1:]):
                    arclength += numpy.linalg.norm(waypoint2_pose[0:3, 3] - waypoint1_pose[0:3, 3])

                # Render a line connecting the interpolated trajectory.
                with self.robot.CreateRobotStateSaver():
                    self.robot.SetActiveDOFs(dof_indices)
                    num_samples = numpy.ceil(arclength / self.resolution)

                    interp_points = list()
                    for t in numpy.linspace(0, self.traj.GetDuration(), num_samples):
                        # Interpolate the joint values.
                        waypoint = self.traj.Sample(t)
                        q = config_spec.ExtractJointValues(waypoint, self.robot, dof_indices)

                        # Compute the end-effector pose using the robot's forward-kinematics.
                        self.robot.SetActiveDOFValues(q)
                        hand_pose = self.robot.GetActiveManipulator().GetEndEffectorTransform()
                        interp_points.append(hand_pose[0:3, 3])

                    interp_points = numpy.array(interp_points)
                    if len(interp_points) > 0:
                           self.lines_handle = self.env.drawlinestrip(interp_points, self.linewidth, self.color)

    def __exit__(self, *exc_info):
        with self.env:
            if self.render_waypoints:
                self.env.Remove(self.sphere_handle)

            if self.render_interpolation:
                del self.lines_handle
