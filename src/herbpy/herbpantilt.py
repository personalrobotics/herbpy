import logging, numpy, openravepy, rospy
import prpy
from prpy.base.wam import WAM

class HERBPantilt(WAM):
    def __init__(self, sim, owd_namespace):
        # FIXME: We don't build the IK database because ikfast fails with a
        # compilation error on the pantilt. This should actually be LookAt3D.
        WAM.__init__(self, sim, owd_namespace, iktype=None)

    def CloneBindings(self, parent):
        WAM.CloneBindings(self, parent)

    def FollowHand(self, traj, manipulator):
        """Modify a trajectory to make the head follow an end-effector.
        The input trajectory must not include any of the head's DOFs and will
        be modified to include the head. This is implemented by appending the
        appropriate head joint angles to the existing waypoints in the
        trajectory. This means that the head is only guaranteed to perfectly
        track the end-effector at the input waypoints.
        @param traj input trajectory that does not include the head DOFs
        @param manipulator end-effector to track
        """
        robot = self.GetRobot()
        traj_config_spec = traj.GetConfigurationSpecification()
        head_config_spec = self.GetArmConfigurationSpecification()
        arm_indices = manipulator.GetArmIndices()
        head_indices = self.GetArmIndices()

        # Construct a path of head joint values that starts at the current
        # configuration and tracks the arm at each waypoint. Note that there
        # may be no IK solution at some waypoints.
        head_path = list()
        head_path.append(robot.GetDOFValues(head_indices))
        last_ik_index = 0

        with robot.GetEnv():
            with robot:
                for i in xrange(1, traj.GetNumWaypoints()):
                    traj_waypoint = traj.GetWaypoint(i)
                    arm_dof_values = traj_config_spec.ExtractJointValues(traj_waypoint, robot, arm_indices)
                    # Compute the position of the right arm through the FK.
                    manipulator.SetDOFValues(arm_dof_values)
                    hand_pose = manipulator.GetEndEffectorTransform()

                    # This will be None if there is no IK solution.
                    head_dof_values = self.FindIK(hand_pose[0:3, 3])
                    head_path.append(head_dof_values)
                    if head_dof_values is not None:
                        final_ik_index = i

        # Propagate the last successful IK solution to all following waypoints.
        # This lets us avoid some edge cases during interpolation.
        for i in xrange(final_ik_index + 1, traj.GetNumWaypoints()):
            head_path[i] = head_path[final_ik_index]

        # Interpolate to fill in IK failures. This is guaranteed to succeed because
        # the first and last waypoints are always valid.
        for i in xrange(1, traj.GetNumWaypoints()):
            # TODO: Fix timestamps on waypoints in MacTrajectory so we can properly
            # interpolate between waypoints.
            if head_path[i] is None:
                head_path[i] = head_path[i - 1]

        # Append the head DOFs to the input trajectory.
        merged_config_spec = traj_config_spec + head_config_spec
        openravepy.planningutils.ConvertTrajectorySpecification(traj, merged_config_spec)

        for i in xrange(0, traj.GetNumWaypoints()):
            waypoint = traj.GetWaypoint(i)
            merged_config_spec.InsertJointValues(waypoint, head_path[i], robot, head_indices, 0)
            traj.Insert(i, waypoint, True)

    def LookAt(self, target, **kw_args):
        """Look at a point in the world frame.
        Create and, optionally, execute a two waypoint trajectory that starts
        at the current configuration and moves to an IK solution that is
        looking at \a target.
        @param target point in the world frame.
        @param **kw_args keyword arguments passed to \ref MoveTo
        @return pantilt trajectory
        """
        dof_values = self.FindIK(target)
        if dof_values is not None:
            return self.MoveTo(dof_values, **kw_args)
        else:
            raise openravepy.openrave_exception('There is no IK solution available.')

    def MoveTo(self, target_dof_values, execute=True, **kw_args):
        """Move to a target configuration.
        Create and, optionally, execute a two waypoint trajectory that starts
        in the current configuration and moves to \a target_dof_values.
        @param target_dof_values desired configuration
        @param execute optionally execute the trajectory
        @param **kw_args keyword arguments passed to \p robot.ExecuteTrajectory
        @return pantilt trajectory
        """
        raise NotImplementedError('The head is currently disabled under ros_control.')
        # Update the controllers to get new joint values.
        robot = self.GetRobot()
        with robot.GetEnv():
            robot.GetController().SimulationStep(0)
            current_dof_values = self.GetDOFValues()

        config_spec = self.GetArmConfigurationSpecification()
        traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), 'GenericTrajectory')
        traj.Init(config_spec)
        traj.Insert(0, current_dof_values, config_spec)
        traj.Insert(1, target_dof_values, config_spec)

        # Optionally exeucute the trajectory.
        if execute:
            return self.GetRobot().ExecutePath(traj, **kw_args)
        else:
            return traj

    def FindIK(self, target):
        """Find an IK solution that is looking at a desired position.
        @param target target position
        @return IK solution
        """
        ik_params = openravepy.IkParameterization(target, openravepy.IkParameterization.Type.Lookat3D)
        return self.ikmodel.manip.FindIKSolution(ik_params, 0)

