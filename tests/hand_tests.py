#!/usr/bin/env python
PKG = 'herbpy'
import roslib; roslib.load_manifest(PKG)
import numpy, unittest
import herbpy

env, robot = herbpy.initialize(sim=True)

class BarrettHandTest(unittest.TestCase):
    def setUp(self):
        self._env, self._robot = env, robot
        self._wam = robot.right_arm
        self._hand = self._wam.hand
        self._indices = numpy.array(sorted(self._wam.GetChildDOFIndices()))
        self._num_dofs = len(self._indices)

    def test_GetIndices_ReturnsIndices(self):
        numpy.testing.assert_array_equal(self._hand.GetIndices(), self._indices)

    def test_GetDOFValues_SetsValues(self):
        expected_values = numpy.array([ 0.1, 0.2, 0.3, 0.4 ])
        self._robot.SetDOFValues(expected_values, self._indices)
        numpy.testing.assert_array_almost_equal(self._hand.GetDOFValues(), expected_values)

    def test_SetDOFValues_SetsValues(self):
        expected_values = numpy.array([ 0.1, 0.2, 0.3, 0.4 ])
        self._hand.SetDOFValues(expected_values)
        numpy.testing.assert_array_almost_equal(self._robot.GetDOFValues(self._indices), expected_values)

    def test_SetDOFValues_IncorrectSizeThrows(self):
        self.assertRaises(Exception, self._hand.SetDOFValues, ([ 0.1, 0.2, 0.3 ],))
        self.assertRaises(Exception, self._hand.SetDOFValues, ([ 0.1, 0.2, 0.3, 0.4, 0.5 ],))

    def test_MoveHand_NoTimeoutWaitsForController(self):
        before = numpy.zeros(4)
        after = numpy.array([ 0.1, 0.0, 0.0, 0.0 ])
        self._robot.SetDOFValues(before, self._indices)
        self._hand.MoveHand(f1=after[0], timeout=None)
        numpy.testing.assert_array_almost_equal(self._robot.GetDOFValues(self._indices), after)

    def test_MoveHand_MovesOneFinger(self):
        for i in xrange(self._num_dofs):
            before = numpy.zeros(self._num_dofs)
            after = numpy.zeros(self._num_dofs)
            after[i] = 0.1

            desired = [ None ] * self._num_dofs
            desired[i] = after[i]

            self._robot.SetDOFValues(before, self._indices)
            self._hand.MoveHand(*desired)
            self._robot.WaitForController(0)
            numpy.testing.assert_array_almost_equal(self._robot.GetDOFValues(self._indices), after)

    def test_MoveHand_MovesAllFingers(self):
        before = numpy.zeros(self._num_dofs)
        after = numpy.arange(self._num_dofs)

        self._robot.SetDOFValues(before, self._indices)
        self._hand.MoveHand(*after)
        self._robot.WaitForController(0)
        numpy.testing.assert_array_almost_equal(self._robot.GetDOFValues(self._indices), after)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_hand', BarrettHandTest)
