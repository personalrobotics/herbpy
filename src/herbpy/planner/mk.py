import cbirrt, logging, numpy, openravepy, time
import prrave.tsr
import planner

def DoNothing(robot):
    return numpy.zeros(robot.GetActiveDOF())

# Based on Moslem Kazemi's code from ARM-S.
def JointLimitAvoidance(robot, limit_tolerance=0.2, gain=50):
    q = robot.GetActiveDOFValues()
    q_min, q_max = robot.GetDOFLimits(robot.GetActiveDOFIndices())

    num_dofs = robot.GetActiveDOF()
    q_dot = numpy.zeros(num_dofs)
    for i in xrange(num_dofs):
        max_limit_dist = q_max[i] - q[i]
        min_limit_dist = q_min[i] - q[i]

        if max_limit_dist < limit_tolerance:
            q_dot[i] = -gain * (max_limit_dist - limit_tolerance) ** 2
        elif min_limit_dist > -limit_tolerance:
            q_dot[i] =  gain * (min_limit_dist + limit_tolerance) ** 2
        else:
            q_dot[i] = 0

    return q_dot

class MKPlanner(planner.Planner):
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot

    def GetName(self):
        return 'mk'

    def GetStraightVelocity(self, manip, velocity, initial_hand_pose, nullspace_fn, step_size):
        current_hand_pose = manip.GetEndEffectorTransform()
        initial_position = initial_hand_pose[0:3, 3]
        current_position = current_hand_pose[0:3, 3]

        # Simulate a position goal step_size distance further along the velocity vector.
        moved_already = velocity * numpy.dot(current_position - initial_position, velocity)
        desired_position = initial_position + moved_already + velocity * step_size
        error_pos = desired_position - current_position

        # Append the desired quaternion to create the error vector. There is a
        # sign ambiguity on quaternions, so we'll always choose the shortest path.
        initial_ori = openravepy.quatFromRotationMatrix(initial_hand_pose)
        current_ori = openravepy.quatFromRotationMatrix(current_hand_pose)
        choices_ori = [ current_ori - initial_ori, current_ori + initial_ori ]
        error_ori = min(choices_ori, key=lambda q: numpy.linalg.norm(q))

        # Jacobian pseudo-inverse.
        jacobian_spatial = manip.CalculateJacobian()
        jacobian_angular = manip.CalculateRotationJacobian()
        jacobian = numpy.vstack((jacobian_spatial, jacobian_angular))
        jacobian_pinv = numpy.linalg.pinv(jacobian)

        # Null-space projector
        nullspace_projector = numpy.eye(jacobian.shape[1]) - numpy.dot(jacobian_pinv, jacobian)
        nullspace_goal = nullspace_fn(self.robot)
        pose_error = numpy.hstack((error_pos, error_ori))
        return numpy.dot(jacobian_pinv, pose_error) + numpy.dot(nullspace_projector, nullspace_goal)

    def PlanToEndEffectorOffset(self, direction, distance, nullspace=JointLimitAvoidance,
                                timelimit=2.5, step_size=0.001,
                                position_tolerance=0.01, angular_tolerance=0.15, **kw_args):
        direction  = numpy.array(direction, dtype='float')
        direction /= numpy.linalg.norm(direction)

        with self.env:
            with self.robot.CreateRobotStateSaver():
                manip = self.robot.GetActiveManipulator()
                traj = openravepy.RaveCreateTrajectory(self.env, '')
                traj.Init(manip.GetArmConfigurationSpecification())

                active_dof_indices = manip.GetArmIndices()
                limits_lower, limits_upper = self.robot.GetDOFLimits(active_dof_indices)
                initial_pose = manip.GetEndEffectorTransform()
                q = self.robot.GetDOFValues(active_dof_indices)
                traj.Insert(0, q)

                start_time = time.time()
                current_distance = 0.0
                while current_distance < distance:
                    # Check for a timeout.
                    current_time = time.time()
                    if timelimit is not None and current_time - start_time > timelimit:
                        raise planner.PlanningError('Reached time limit.')

                    # Compute joint velocities using the Jacobian pseudoinverse.
                    q_dot = self.GetStraightVelocity(manip, direction, initial_pose, nullspace, step_size)
                    q += q_dot
                    self.robot.SetDOFValues(q, active_dof_indices)
                    traj.Insert(traj.GetNumWaypoints(), q)

                    # Check for collisions.
                    if self.env.CheckCollision(self.robot):
                        raise planner.PlanningError('Encountered collision.')
                    elif self.robot.CheckSelfCollision():
                        raise planner.PlanningError('Encountered self-collision.')
                    # Check for joint limits.
                    elif not (limits_lower < q).all() or not (q < limits_upper).all():
                        raise planner.PlanningError('Encountered joint limit during Jacobian move.')

                    # Check our distance from the constraint.
                    current_pose = manip.GetEndEffectorTransform()
                    a = initial_pose[0:3, 3]
                    p = current_pose[0:3, 3]
                    orthogonal_proj = (a - p) - numpy.dot(a - p, direction) * direction
                    if numpy.linalg.norm(orthogonal_proj) > position_tolerance:
                        raise planner.PlanningError('Deviated from a straight line constraint.')

                    # Check our orientation against the constraint.
                    offset_pose = numpy.dot(numpy.linalg.inv(current_pose), initial_pose)
                    offset_angle = openravepy.axisAngleFromRotationMatrix(offset_pose)
                    if numpy.linalg.norm(offset_angle) > angular_tolerance:
                        raise planner.PlanningError('Deviated from orientation constraint.')

                    # Check if we've exceeded the maximum distance by projecting our
                    # displacement along the direction.
                    hand_pose = manip.GetEndEffectorTransform()
                    displacement = hand_pose[0:3, 3] - initial_pose[0:3, 3]
                    current_distance = numpy.dot(displacement, direction)

        return traj
