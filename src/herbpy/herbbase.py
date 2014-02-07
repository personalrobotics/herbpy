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

class HerbBase(MobileBase):

    def __init___(self, sim, robot):
        MobileBase.__init__(self, sim=sim, robot=robot)

    def CloneBindings(self, parent):
        MobileBase.CloneBindings(self, parent)

    def Forward(self, meters, timeout=None):
        """
        Drives the robot forward the desired distance
        Note: Only implemented in simulation. Derived robots should implement this method.
        @param meters the distance to drive the robot
        @param timout duration to wait for execution
        """
        if self.simulated:
            MobileBase.Forward(self, meters, timeout=timeout)
        else:
            self.robot.DriveSegway(self.robot, meters, timeout=timeout)

    def Rotate(self, angle_rad, timeout=None):
        """
        Rotates the robot the desired distance
        @param angle_rad the number of radians to rotate
        @param timeout duration to wait for execution
        """
        if self.simulated:
            MobileBase.Rotate(self, angle_rad, timeout=timeout)
        else:
            self.robot.RotateSegway(self.robot, angle_rad, timeout=timeout)
