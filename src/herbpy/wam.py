import logging, math, numpy, openravepy, threading, time
import exceptions, util
from planner import PlanningError 
from util import Deprecated

logger = logging.getLogger('herbpy')

class WAM(openravepy.Robot.Manipulator):
    def SetStiffness(manipulator, stiffness):
        """
        Set the WAM's stiffness. This enables or disables gravity compensation.
        @param stiffness value between 0.0 and 1.0
        """
        if not manipulator.simulated:
            manipulator.controller.SendCommand('SetStiffness {0:f}'.format(stiffness))

    def Servo(manipulator, velocities):
        """
        Servo with an instantaneous vector joint velocities.
        @param joint velocities
        """
        num_dof = len(manipulator.GetArmIndices())
        if len(velocities) != num_dof:
            raise ValueError('Incorrect number of joint velocities. Expected {0:d}; got {0:d}.'.format(
                             num_dof, len(velocities)))

        if not manipulator.simulated:
            manipulator.controller.SendCommand('Servo ' + ' '.join([ str(qdot) for qdot in velocities ]))
        else:
            manipulator.controller.Reset(0)
            manipulator.servo_simulator.SetVelocity(velocities)

    def ServoTo(manipulator, target, duration, timeStep = 0.05, collisionChecking= True):
        """
        Servo's the WAM to the target taking the duration passed to it
        @param target dofs
        @param duration of the full servo
        @param timeStep
        @param collisionChecking
        """
        steps = int(math.ceil(duration/timeStep))
        original_dofs = manipulator.parent.GetDOFValues(manipulator.GetArmIndices())
        velocity = numpy.array(target-manipulator.parent.GetDOFValues(manipulator.GetArmIndices()))
        velocities = v/steps#[v/steps for v in velocity]
        inCollision = False 
        if collisionChecking==True:
            inCollision = manipulator.CollisionCheck(target)
        if inCollision == False:       
            for i in range(1,steps):
                manipulator.Servo(velocities)
                time.sleep(timeStep)
            manipulator.Servo([0] * len(manipulator.GetArmIndices()))
            new_dofs = manipulator.parent.GetDOFValues(manipulator.GetArmIndices())
            return True
        return False

    def SetVelocityLimits(manipulator, velocity_limits, min_accel_time):
        """
        Change the OWD velocity limits and acceleration time.
        @param velocity_limits individual joint velocity limits
        @param min_accel_time acceleration limit
        """
        velocity_limits = numpy.array(velocity_limits, dtype='float')
        num_dofs = len(manipulator.GetArmIndices())
        if len(velocity_limits) != num_dofs:
            logger.error('Incorrect number of velocity limits; expected {0:d}, got {1:d}.'.format(
                          num_dofs, len(velocity_limits)))
            return False

        # Update the OpenRAVE limits.
        with manipulator.parent.GetEnv():
            active_indices = manipulator.GetArmIndices()

            or_velocity_limits = manipulator.parent.GetDOFVelocityLimits()
            or_velocity_limits[active_indices] = velocity_limits
            manipulator.parent.SetDOFVelocityLimits(or_velocity_limits)

            or_accel_limits = manipulator.parent.GetDOFAccelerationLimits()
            or_accel_limits[active_indices] = velocity_limits / min_accel_time
            manipulator.parent.SetDOFAccelerationLimits(or_accel_limits)

        # Update the OWD limits.
        if not manipulator.simulated:
            args  = [ 'SetSpeed' ]
            args += [ str(min_accel_time) ]
            args += [ str(velocity) for velocity in velocity_limits ]
            args_str = ' '.join(args)
            manipulator.controller.SendCommand(args_str)
        return True

    def GetTrajectoryStatus(manipulator):
        '''
        Gets the status of the current (or previous) trajectory executed by the
        controller.
        '''
        if not manipulator.simulated:
            return manipulator.controller.SendCommand('GetStatus')
        else:
            if manipulator.controller.IsDone():
                return 'done'
            else:
                return 'active'

    def ClearTrajectoryStatus(manipulator):
        '''
        Clears the current trajectory execution status.
        '''
        if not manipulator.simulated:
            manipulator.controller.SendCommand('ClearStatus')

    def SetActive(manipulator):
        '''
        Sets this as the active manipulator and updates the active DOF indices.
        '''
        manipulator.parent.SetActiveManipulator(manipulator)
        manipulator.parent.SetActiveDOFs(manipulator.GetArmIndices())

    def GetDOFValues(manipulator):
        return manipulator.parent.GetDOFValues(manipulator.GetArmIndices())

    def SetDOFValues(manipulator, dof_values,
                     limits_action=openravepy.KinBody.CheckLimitsAction.CheckLimits):
        manipulator.parent.SetDOFValues(dof_values, manipulator.GetArmIndices(), limits_action)

    def MoveUntilTouch(manipulator, direction, distance, max_force=5, **kw_args):
        """
        Execute a straight move-until-touch action. This action stops when a
        sufficient force is is felt or the manipulator moves the maximum distance.
        @param direction unit vector for the direction of motion in the world frame
        @param distance maximum distance in meters
        @param max_force maximum force in Newtons
        @param execute optionally execute the trajectory
        @param **kw_args planner parameters
        @return felt_force flag indicating whether we felt a force.
        """
        with manipulator.parent.GetEnv():
            manipulator.parent.GetController().SimulationStep(0)

            # Compute the expected force direction in the hand frame.
            direction = numpy.array(direction)
            hand_pose = manipulator.GetEndEffectorTransform()
            force_direction = numpy.dot(hand_pose[0:3, 0:3].T, -direction)

            with manipulator.parent:
                traj = manipulator.PlanToEndEffectorOffset(direction, distance, execute=False, **kw_args)
                traj = manipulator.parent.BlendTrajectory(traj)
                traj = manipulator.parent.RetimeTrajectory(traj, stop_on_ft=True, force_direction=force_direction,
                                                           force_magnitude=max_force, torque=[100,100,100])

        # TODO: Use a simulated force/torque sensor in simulation.
        try:
            manipulator.TareForceTorqueSensor()
            manipulator.parent.ExecuteTrajectory(traj, execute=True, retime=False, blend=False)
            return False
        # Trajectory is aborted by OWD because we felt a force.
        except exceptions.TrajectoryAborted:
            return True

    def PlanToNamedConfiguration(manipulator, name, **kw_args):
        config_inds = numpy.array(manipulator.parent.configs[name]['dofs'])
        config_vals = numpy.array(manipulator.parent.configs[name]['vals'])

        if len(config_inds) == 0:
            raise Exception('Failed to find named config: %s'%name)
        

        # TODO: Hacky. Can we do this better?
        # Need to parse out left and right manipulator indices
        # Also do we want to parse out head?
        arm_indices = numpy.array(manipulator.GetArmIndices())
        arm_indices.sort()

        arm_vals= []
        arm_inds = []


        for dof, val in zip(config_inds, config_vals):
            if dof in arm_indices:
                arm_inds.append(dof)
                arm_vals.append(val)

        traj = None
        if len(arm_inds) > 0:
            manipulator.parent.SetActiveDOFs(arm_inds)
            traj = manipulator.PlanToConfiguration(arm_vals, **kw_args)
       
        return traj

    # Deprecated methods that were moved to the hand class.
    @Deprecated('Use GetDOFValues instead.')
    def GetArmDOFValues(manipulator):
        return manipulator.GetDOFValues()

    @Deprecated('Use SetDOFValues instead.')
    def SetArmDOFValues(manipulator, dof_values):
        return manipulator.SetDOFValues(dof_values)

    @Deprecated('Use hand.OpenHand instead.')
    def OpenHand(manipulator, *args, **kw_args):
        return manipulator.hand.OpenHand(*args, **kw_args)

    @Deprecated('Use hand.CloseHand instead.')
    def CloseHand(manipulator, spread=None, timeout=None):
        return manipulator.hand.CloseHand(*args, **kw_args)

    @Deprecated('Use hand.MoveHand instead.')
    def MoveHand(manipulator, *args, **kw_args):
        return manipulator.hand.MoveHand(*args, **kw_args)

    @Deprecated('Use hand.GetForceTorque instead.')
    def GetForceTorque(manipulator, *args, **kw_args):
        return manipulator.hand.GetForceTorque(*args, **kw_args)

    @Deprecated('Use hand.TareForceTorqueSensor instead.')
    def TareForceTorqueSensor(manipulator, *args, **kw_args):
        return manipulator.hand.TareForceTorqueSensor()

    @Deprecated('Use hand.GetStrain.')
    def GetStrain(manipulator, *args, **kw_args):
        return manipulator.hand.GetStrain(*args, **kw_args)

    @Deprecated('Use hand.GetBreakaway.')
    def GetBreakaway(manipulator, *args, **kw_args):
        return manipulator.hand.GetBreakaway(*args, **kw_args)

    @Deprecated('Use hand.LookAtHand.')
    def LookAtHand(manipulator, **kw_args):
        return manipulator.hand.LookAtHand(*args, **kw_args)
