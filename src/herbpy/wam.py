#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import logging
import numbers
import numpy
import openravepy
import warnings
from prpy import exceptions
from prpy.base.manipulator import Manipulator
from prpy.clone import Clone

logger = logging.getLogger('wam')

class WAM(Manipulator):
    def __init__(self, sim, namespace='',
                 iktype=openravepy.IkParameterization.Type.Transform6D):
        Manipulator.__init__(self)

        self.simulated = sim
        self._iktype = iktype
        self.namespace = namespace

        if iktype is not None:
            self._SetupIK(iktype)

        if sim:
            from prpy.simulation import ServoSimulator
            self.servo_simulator = ServoSimulator(
                self, rate=20, watchdog_timeout=0.1)

    def IsSimulated(self):
        return self.simulated

    def GetJointNames(self):
        return [self.namespace + '/' + j for j in
                ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']]

    def CloneBindings(self, parent):
        Manipulator.CloneBindings(self, parent)

        self.simulated = True
        self._iktype = parent._iktype

        if self._iktype is not None:
            self._SetupIK(self._iktype)

    def _SetupIK(self, iktype):
        from openravepy.databases.inversekinematics import InverseKinematicsModel

        robot = self.GetRobot()
        self.ikmodel = InverseKinematicsModel(robot=robot, manip=self,
                                              iktype=iktype)
        if not self.ikmodel.load():
            self.ikmodel.generate(iktype=iktype, precision=4,
                                  freeindices=[ self.GetIndices()[2] ])
            self.ikmodel.save()

    def SetStiffness(self, stiffness):
        """Set the WAM's stiffness.
        Stiffness False/0 is gravity compensation and stiffness True/1 is position
        control. Values between 0 and 1 are experimental.
        @param stiffness boolean or decimal value between 0.0 and 1.0
        """
        self.GetRobot().SetStiffness(stiffness, manip=self)


    def SetTrajectoryExecutionOptions(self, traj, stop_on_stall=False,
            stop_on_ft=False, force_magnitude=None, force_direction=None,
            torque=None):
        """Set OWD's trajectory execution options on trajectory.
        @param stop_on_stall aborts the trajectory if the arm stalls
        @param stop_on_ft aborts the trajectory if the force/torque
                          sensor reports a force or torque that exceeds
                          threshold specified by the force_magnitude,
                          force_direction, and torque options
        @param force_magnitude force threshold value, in Newtons
        @param force_direction unit vector in the force/torque coordinate
                               frame, which is int he same orientation as the
                               hand frame.
        @param torque vector of the three torques
        """
        raise NotImplementedError('OWD execution options not supported under ros_control')

    def Servo(self, velocities):
        """Servo with a vector of instantaneous joint velocities.
        @param velocities joint velocities, in radians per second
        """
        num_dof = len(self.GetArmIndices())
        if len(velocities) != num_dof:
            raise ValueError('Incorrect number of joint velocities. '
                             'Expected {0:d}; got {0:d}.'.format(
                             num_dof, len(velocities)))

        if not self.simulated:
            raise NotImplementedError('Servoing and velocity control not yet'
                                      'supported under ros_control.')

        else:
            self.servo_simulator.SetVelocity(velocities)

    def ServoTo(self, target, duration, timeStep=0.05, collisionChecking=True):
        """Servo the arm towards a target configuration over some duration.
        Servos the arm towards a target configuration with a constant joint
        velocity. This function uses the \ref Servo command to control the arm
        and must be called repeatidly until the arm reaches the goal. If \tt
        collisionChecking is enabled, then the servo will terminate and return
        False if a collision is detected with the simulation environment.
        @param target desired configuration
        @param duration duration in seconds
        @param timestep period of the control loop, in seconds
        @param collisionchecking check collisions in the simulation environment
        @return whether the servo was successful
        """
        steps = int(math.ceil(duration/timeStep))
        original_dofs = self.GetRobot().GetDOFValues(self.GetArmIndices())
        velocity = numpy.array(target - self.GetRobot().GetDOFValues(self.GetArmIndices()))
        velocities = [v/steps for v in velocity]
        inCollision = False
        if collisionChecking:
            inCollision = self.CollisionCheck(target)

        if not inCollision:
            for i in range(1,steps):
                import time
                self.Servo(velocities)
                time.sleep(timeStep)
            self.Servo([0] * len(self.GetArmIndices()))
            # new_dofs = self.GetRobot().GetDOFValues(self.GetArmIndices())
            return True
        else:
            return False

    def GetVelocityLimits(self, openrave=None, owd=None):
        """Get the OpenRAVE and OWD joint velocity limits.
        This function checks both the OpenRAVE and OWD joint velocity limits.
        If they do not match, a warning is printed and the minimum value is
        returned.
        @param openrave flag to set the OpenRAVE velocity limits
        @param owd flag to set the OWD velocity limits
        @return list of velocity limits, in radians per second
        """
        if openrave is not None or owd is not None:
            warnings.warn(
                'The "openrave" and "owd" flags are deprecated in'
                ' GetVelocityLimits and will be removed in a future version.',
                DeprecationWarning)

        return Manipulator.GetVelocityLimits(self)

    def SetVelocityLimits(self, velocity_limits, min_accel_time,
                          openrave=True, owd=None):
        """Change the OpenRAVE and OWD joint velocity limits.
        Joint velocities that exceed these limits will trigger a velocity fault.
        @param velocity_limits vector of joint velocity limits in radians per second
        @param min_accel_time minimum acceleration time used to compute acceleration limits
        @param openrave flag to set the OpenRAVE velocity limits
        @param owd flag to set the OWD velocity limits
        """
        # TODO implement in ros_control
        if owd is not None:
            warnings.warn(
                'The "owd" flag is deprecated in'
                ' SetVelocityLimits and will be removed in a future version.',
                DeprecationWarning)

        # Update the OpenRAVE limits.
        if openrave:
            Manipulator.SetVelocityLimits(self, velocity_limits, min_accel_time)

    def GetTrajectoryStatus(manipulator):
        """Gets the status of the current (or previous) trajectory executed by OWD.
        @return status of the current (or previous) trajectory executed
        """
        raise NotImplementedError('GetTrajectoryStatus not supported on manipulator.'
                                  ' Use returned TrajectoryFuture instead.')

    def ClearTrajectoryStatus(manipulator):
        """Clears the current trajectory execution status.
        This resets the output of \ref GetTrajectoryStatus.
        """
        raise NotImplementedError('ClearTrajectoryStatus not supported on manipulator.')

    def MoveUntilTouch(self,
                       direction,
                       distance,
                       max_distance=1.0,
                       max_force=5.0,
                       max_torque=2.75,
                       ignore_collisions=None,
                       velocity_limit_scale=0.25,
                       **kw_args):
        """Execute a straight move-until-touch action.
        This action stops when a sufficient force is is felt or the manipulator
        moves the maximum distance. The motion is considered successful if the
        end-effector moves at least distance. In simulation, a move-until-touch
        action proceeds until the end-effector collids with the environment.

        @param direction unit vector for the direction of motion in the world frame
        @param distance minimum distance in meters
        @param max_distance maximum distance in meters
        @param max_force maximum force in Newtons
        @param max_torque maximum torque in Newton-Meters
        @param ignore_collisions collisions with these objects are ignored when
        planning the path, e.g. the object you think you will touch
        @param velocity_limit_scale A multiplier to use to scale velocity limits
        when executing MoveUntilTouch ( < 1 in most cases).
        @param **kw_args planner parameters
        @return felt_force flag indicating whether we felt a force.
        """
        from contextlib import nested
        from openravepy import CollisionReport, KinBody, Robot, RaveCreateTrajectory
        from prpy.planning.exceptions import CollisionPlanningError

        delta_t = 0.01

        robot = self.GetRobot()
        env = robot.GetEnv()
        dof_indices = self.GetArmIndices()

        direction = numpy.array(direction, dtype='float')

        if ignore_collisions is None:
            ignore_collisions = []

        with env:
            # Disable the KinBodies listed in ignore_collisions. We backup the
            # "enabled" state of all KinBodies so we can restore them later.
            body_savers = [
                body.CreateKinBodyStateSaver() for body in ignore_collisions]
            robot_saver = robot.CreateRobotStateSaver(
                Robot.SaveParameters.ActiveDOF |
                Robot.SaveParameters.ActiveManipulator |
                Robot.SaveParameters.LinkTransformation)

            with robot_saver, nested(*body_savers):
                self.SetActive()
                robot_cspec = robot.GetActiveConfigurationSpecification()

                for body in ignore_collisions:
                    body.Enable(False)

                path = robot.PlanToEndEffectorOffset(direction=direction,
                                                     distance=distance,
                                                     max_distance=max_distance,
                                                     **kw_args)

        # Execute on the real robot by loading the right controller and
        # setting the appropriate force/torque limits
        if not self.simulated:
            from actionlib import SimpleActionClient
            from pr_control_msgs.msg import (
                SetForceTorqueThresholdAction as FTThresholdAction,
                SetForceTorqueThresholdActionGoal as FTThresholdGoal)
            from rospy import Duration

            # determine controller name from manipulator
            ns = str.translate(self.namespace, '/')
            if ns is '':
                controller_name = 'move_until_touch_controller'
            else:
                controller_name = ns + '_move_until_touch_controller'

            # load controller before setting threshold
            self.controller_manager.request([controller_name]).switch()

            # connect to action server
            server_name = '/' + controller_name + '/set_forcetorque_threshold',
            client = SimpleActionClient(server_name, FTThresholdAction)
            one_second = Duration(1.0)
            if not client.wait_for_server(one_second):  # TODO timeout?
                raise RuntimeError('Could not connect to action server \'{}\''
                                   .format(server_name))
            # set force/torque threshold
            goal = FTThresholdGoal()
            goal.force_threshold = max_force
            goal.torque_threshold = max_torque
            client.send_goal(goal)
            client.wait_for_result(one_second)
            result = client.get_result()
            if not result.success:
                raise RuntimeError('Failed to set force/torque threshold: {}'
                                   .format(result.message))

            # execute movement
            robot.ExecutePath(path, move_until_touch=True, **kw_args)

        # Forward-simulate the motion until it hits an object.
        else:
            traj = robot.PostProcessPath(path)
            is_collision = False

            traj_cspec = traj.GetConfigurationSpecification()
            new_traj = RaveCreateTrajectory(env, '')
            new_traj.Init(traj_cspec)

            robot_saver = robot.CreateRobotStateSaver(
                Robot.SaveParameters.LinkTransformation)

            with env, robot_saver:
                for t in numpy.arange(0, traj.GetDuration(), delta_t):
                    waypoint = traj.Sample(t)

                    dof_values = robot_cspec.ExtractJointValues(
                        waypoint, robot, dof_indices, 0)
                    self.SetDOFValues(dof_values)

                    # Terminate if we detect collision with the environment.
                    report = CollisionReport()
                    if env.CheckCollision(robot, report=report):
                        logger.info('Terminated from collision: %s',
                            str(CollisionPlanningError.FromReport(report)))
                        is_collision = True
                        break
                    elif robot.CheckSelfCollision(report=report):
                        logger.info('Terminated from self-collision: %s',
                            str(CollisionPlanningError.FromReport(report)))
                        is_collision = True
                        break

                    # Build the output trajectory that stops in contact.
                    if new_traj.GetNumWaypoints() == 0:
                        traj_cspec.InsertDeltaTime(waypoint, 0.)
                    else:
                        traj_cspec.InsertDeltaTime(waypoint, delta_t)

                    new_traj.Insert(new_traj.GetNumWaypoints(), waypoint)

            if new_traj.GetNumWaypoints() > 0:
                robot.ExecuteTrajectory(new_traj)

            return is_collision
