#!/usr/bin/env python
PKG = 'herbpy'
import roslib; roslib.load_manifest(PKG)
import numpy, unittest
import herbpy

env, robot = herbpy.initialize(sim=True)

class WamTest(unittest.TestCase):
    def setUp(self):
        self._env, self._robot = env, robot
        self._wam = robot.right_arm
        self._indices = self._wam.GetArmIndices()
        self._num_dofs = len(self._indices)
        self._zeros = numpy.zeros(self._num_dofs)
        self._ones = numpy.ones(self._num_dofs)
        self._small_ones = numpy.ones(self._num_dofs - 1)
        self._large_ones = numpy.ones(self._num_dofs + 1)

    def test_SetStiffness_DoesNotThrow(self):
        self._wam.SetStiffness(0.0)
        self._wam.SetStiffness(0.5)
        self._wam.SetStiffness(1.0)

    def test_SetStiffness_InvalidStiffnessThrows(self):
        self.assertRaises(Exception, self._wam.SetStiffness, (-0.2,))
        self.assertRaises(Exception, self._wam.SetStiffness, ( 1.2,))

    def test_Servo_IncorrectSizeThrows(self):
        self._wam.Servo(0.1 * self._ones)

    def test_Servo_IncorrectSizeThrows(self):
        num_dofs = len(self._wam.GetArmIndices())
        self.assertRaises(Exception, self._wam.Servo, (0.1 * self._small_ones,))
        self.assertRaises(Exception, self._wam.Servo, (0.1 * self._large_ones,))

    def test_Servo_ExceedsVelocityLimitThrows(self):
        velocity_limits = self._robot.GetDOFVelocityLimits(self._indices)
        min_accel_time = 0.3
        self.assertRaises(Exception, self._wam.Servo, (+velocity_limits + 0.1 * self._ones, min_accel_time))
        self.assertRaises(Exception, self._wam.Servo, (-velocity_limits - 0.1 * self._ones, min_accel_time))

    def test_Servo_InvalidAccelTimeThrows(self):
        self.assertRaises(Exception, self._wam.Servo, (0.1 * self._ones, -0.1))
        self.assertRaises(Exception, self._wam.Servo, (0.1 * self._ones,  0.0))

    def test_SetVelocityLimits_SetsLimits(self):
        self._wam.SetVelocityLimits(self._ones, 0.3)
        velocity_limits = self._robot.GetDOFVelocityLimits(self._indices)
        numpy.testing.assert_array_almost_equal(velocity_limits, self._ones)

    def test_SetVelocityLimits_IncorrectSizeThrows(self):
        self.assertRaises(Exception, self._wam.SetVelocityLimits, (0.1 * self._small_ones,))
        self.assertRaises(Exception, self._wam.SetVelocityLimits, (0.1 * self._large_ones,))

    def test_SetVelocityLimits_NegativeLimitThrows(self):
        self.assertRaises(Exception, self._wam.SetVelocityLimits, (-self._ones,))

    def test_SetActive_SetsActiveManipulator(self):
        self._robot.SetActiveManipulator('head_wam')
        self._wam.SetActive()
        self.assertEquals(self._robot.GetActiveManipulator(), self._wam)

    def test_SetActive_SetsActiveDOFs(self):
        self._robot.SetActiveDOFs([ 0 ])
        self._wam.SetActive()
        numpy.testing.assert_array_almost_equal(self._robot.GetActiveDOFIndices(), self._indices)

    def test_SetDOFValues_SetsValues(self):
        robot.SetDOFValues(self._zeros, self._indices)
        dof_values = 0.6 * self._ones
        self._wam.SetDOFValues(dof_values)
        numpy.testing.assert_array_almost_equal(self._robot.GetDOFValues(self._indices), dof_values)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_wam', WamTest)
