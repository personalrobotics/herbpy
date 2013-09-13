import logging, numpy, openravepy, rospy
import prpy

class HERBPantilt(prpy.base.WAM):
    def __init__(self, sim, owd_namespace):
        # FIXME: We don't build the IK database because ikfast fails with a
        # compilation error on the pantilt. This should actually be LookAt3D.
        prpy.base.WAM.__init__(self, sim, owd_namespace, None)

    def CloneBindings(self, parent):
        prpy.base.WAM.CloneBindings(self, parent)

    def FollowHand(self, traj, manipulator):
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
        """
        Look at a point in the world frame. This creates and, optionally executes,
        a two-waypoint trajectory that starts at the current configuration and
        moves to a goal configuration found through the neck's inverse kinematics.
        @param target point in the world frame.
        @param execute immediately execute the trajectory
        @return trajectory head trajectory
        """
        dof_values = self.FindIK(target)
        if dof_values is not None:
            return self.MoveTo(dof_values, **kw_args)
        else:
            raise openravepy.openrave_exception('There is no IK solution available.')

    def MoveTo(self, target_dof_values, execute=True, **kw_args):
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
            return self.GetRobot().ExecuteTrajectory(traj, **kw_args)
        else:
            return traj

    def FindIK(self, target):
        ik_params = openravepy.IkParameterization(target, openravepy.IkParameterization.Type.Lookat3D)
        return self.ikmodel.manip.FindIKSolution(ik_params, 0)

