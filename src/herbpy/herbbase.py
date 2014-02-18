#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Jennifer King <jeking@cs.cmu.edu>
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

from prpy.base import MobileBase
import prpy
import numpy, logging, openravepy
logger = logging.getLogger('herbpy')

class HerbBase(MobileBase):
    def __init__(self, sim, robot):
        MobileBase.__init__(self, sim=sim, robot=robot)
        self.controller = robot.AttachController(name=robot.GetName(),
                                                 args='SegwayController {0:s}'.format('herbpy'),
                                                 dof_indices=[],
                                                 affine_dofs=openravepy.DOFAffine.Transform,
                                                 simulated=sim)

    def CloneBindings(self, parent):
        MobileBase.CloneBindings(self, parent)

    def Forward(self, meters, timeout=None):
        """Drive forward for the desired distance.
        \param distance distance to drive, in meters
        \param timeout time in seconds; pass \p None to block until complete 
        \return base trajectory
        """
        if self.simulated:
            return MobileBase.Forward(self, meters)
        else:
            with prpy.util.Timer("Drive segway"):
                self.controller.SendCommand("Drive " + str(meters))
                is_done = prpy.util.WaitForControllers([ self.controller ], timeout=timeout)

    def Rotate(self, angle_rad, **kw_args):
        """Rotate in place by a desired angle
        \param angle angle to turn, in radians
        \param timeout time in seconds; pass \p None to block until complete 
        \return base trajectory
        """
        if self.simulated:
            MobileBase.Rotate(self, angle_rad, **kw_args)
        else:
            with prpy.util.Timer("Rotate segway"):
                self.controller.SendCommand("Rotate " + str(angle_rad))
                running_controllers = [ self.controller ]
                is_done = prpy.util.WaitForControllers(running_controllers, timeout=timeout)

    def DriveStraightUntilForce(self, direction, velocity=0.1, force_threshold=3.0,
                                max_distance=None, timeout=None, left_arm=True, right_arm=True):
        """
        Drive the base in a direction until a force/torque sensor feels a force. The
        base first turns to face the desired direction, then drives forward at the
        specified velocity. The action terminates when max_distance is reached, the
        timeout is exceeded, or if a force is felt. The maximum distance and timeout
        can be disabled by setting the corresponding parameters to None.
        @param direction forward direction of motion in the world frame
        @param velocity desired forward velocity
        @param force_threshold threshold force in Newtons
        @param max_distance maximum distance in meters
        @param timeout maximum duration in seconds
        @param left_arm flag to use the left force/torque sensor
        @param right_arm flag to use the right force/torque sensor
        @return flag indicating whether the action felt a force
        """
        if self.simulated:
            raise NotImplementedError('DriveStraightUntilForce does not work in simulation')
        else:
            if (self.robot.left_ft_sim and left_arm) or (self.robot.right_ft_sim and right_arm):
                raise Exception('DriveStraightUntilForce does not work with simulated force/torque sensors.')

            with prpy.util.Timer("Drive segway until force"):
                env = self.robot.GetEnv()
                direction = numpy.array(direction, dtype='float')
                direction /= numpy.linalg.norm(direction) 
                manipulators = list()
                if left_arm:
                    manipulators.append(self.robot.left_arm)
                if right_arm:
                    manipulators.append(self.robot.right_arm)

                if not manipulators:
                    logger.warning('Executing DriveStraightUntilForce with no force/torque sensor for feedback.')

                # Rotate to face the right direction.
                with env:
                    robot_pose = self.robot.GetTransform()
                robot_angle = numpy.arctan2(robot_pose[1, 0], robot_pose[0, 0])
                desired_angle = numpy.arctan2(direction[1], direction[0])
                self.Rotate(desired_angle - robot_angle)


                # Soft-tare the force/torque sensors. Tare is too slow.
                initial_force = dict()
                for manipulator in manipulators:
                    force, torque = manipulator.hand.GetForceTorque()
                    initial_force[manipulator] = force
                
                try:
                    felt_force = False
                    start_time = time.time()
                    start_pos = robot_pose[0:3, 3]
                    while True:
                        # Check if we felt a force on any of the force/torque sensors.
                        for manipulator in manipulators:
                            force, torque = manipulator.hand.GetForceTorque()
                            if numpy.linalg.norm(initial_force[manipulator] - force) > force_threshold:
                                return True

                        # Check if we've exceeded the maximum distance.
                        with env:
                            current_pos = self.robot.GetTransform()[0:3, 3]
                        distance = numpy.dot(current_pos - start_pos, direction)
                        if max_distance is not None and distance >= max_distance:
                            return False

                        # Check for a timeout.
                        time_now = time.time()
                        if timeout is not None and time_now - star_time > timeout:
                            return False

                        # Continuously stream forward velocities.
                        self.controller.SendCommand('DriveInstantaneous {0:f} 0 0'.format(velocity))
                finally:
                    # Stop the Segway before returning.
                    self.controller.SendCommand('DriveInstantaneous 0 0 0')
