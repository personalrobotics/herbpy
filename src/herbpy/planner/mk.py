import cbirrt, logging, numpy, openravepy, time
import prrave.tsr
import planner

class MKPlanner(planner.Planner):
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot

    def GetName(self):
        return 'mk'

    def GetStraightVelocity(self, manip, velocity, initial_hand_pose):
        current_hand_pose = manip.GetEndEffectorTransform()

        # Project the position error orthogonal to the velocity of motion. Then
        # add a constant forward force.
        #pos_offset = initial_hand_pose[0:3, 3] - current_hand_pose[0:3, 3]
        #pos_error  = pos_offset - hand_velocity * numpy.dot(pos_offset, hand_velocity)
        error_pos = velocity 

        # Append the desired quaternion to create the error vector. There is a
        # sign ambiguity on quaternions, so we'll always choose the shortest
        # path.
        initial_ori = openravepy.quatFromRotationMatrix(initial_hand_pose)
        current_ori = openravepy.quatFromRotationMatrix(current_hand_pose)
        choices_ori = [ -initial_ori - current_ori, -initial_ori + current_ori,
                        -initial_ori + current_ori, +initial_ori + current_ori ]
        error_ori = min(choices_ori, key=lambda q: numpy.linalg.norm(q))

        # Jacobian pseudo-inverse.
        jacobian_spatial = manip.CalculateJacobian()
        jacobian_angular = manip.CalculateRotationJacobian()
        jacobian = numpy.vstack((jacobian_spatial, jacobian_angular))
        jacobian_pinv = numpy.linalg.pinv(jacobian)

        # TODO: Implement a null-space projector.
        pose_error = numpy.hstack((error_pos, error_ori))
        return numpy.dot(jacobian_pinv, pose_error)

    def PlanToEndEffectorOffset(self, direction, distance, planning_timeout=0.5, step_size=0.01, **kw_args):
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
                    if planning_timeout is not None and current_time - start_time > planning_timeout:
                        raise planner.PlanningError('Reached time limit.')

                    # Compute joint velocities using the Jacobian pseudoinverse.
                    q_dot = self.GetStraightVelocity(manip, direction, initial_pose)
                    q += step_size * q_dot / numpy.linalg.norm(q_dot)
                    self.robot.SetDOFValues(q, active_dof_indices)
                    traj.Insert(traj.GetNumWaypoints(), q)

                    if self.env.CheckCollision(self.robot):
                        raise planner.PlanningError('Encountered collision.')
                    elif self.robot.CheckSelfCollision():
                        raise planner.PlanningError('Encountered self-collision.')
                    elif not (limits_lower < q).all() or not (q < limits_upper).all():
                        raise planner.PlanningError('Encountered joint limit during Jacobian move.')

                    # Check if we've exceeded the maximum distance by projecting our
                    # displacement along the direction.
                    hand_pose = manip.GetEndEffectorTransform()
                    displacement = hand_pose[0:3, 3] - initial_pose[0:3, 3]
                    current_distance = numpy.dot(displacement, direction)

        return traj
