#!/usr/bin/env python
import roslib
import numpy, unittest
import herbpy, prpy, openravepy
from nose.tools import nottest

class RogueTest(unittest.TestCase):
    def setUp(self):
        self.env, self.robot = herbpy.initialize(sim=True)
        self.fuze = prpy.rave.add_object(self.env, 'fuze_bottle', 'objects/fuze_bottle.kinbody.xml')
        self.robot.right_arm.SetActive()
        self.fuze_pose = numpy.eye(4)
        self.fuze_pose[0:3, 3] = [0.8, -0.3, 0.4]
        self.fuze.SetTransform(self.fuze_pose)
        self.fuze.Enable(True)

    def tearDown(self):
        self.env.Destroy()
        openravepy.RaveDestroy()

    def test_Pointing(self):
        self.robot.PointAt([1, 1, 1])
        herbpy.action.Point(self.robot, [1, 1, 1])

    def test_Presenting(self):
        self.robot.PresentAt([0.5, 0, 1])
        herbpy.action.Present(self.robot, [0.5, 0, 1])

    def test_Sweeping(self):
        self.robot.SweepAt([0.5, 0, 0.5], [0.7, 0, 0.5])
        herbpy.action.Sweep(self.robot, [0.7, 0, 0.5], [0.5, 0, 0.5])

    def test_GetPointFrom(self):
        kinbody_coord = prpy.util.GetPointFrom(self.fuze)
        numpy.testing.assert_almost_equal(kinbody_coord, self.fuze_pose[0:3, 3])

        test_array = numpy.array([0, 1, 2])
        numpyarray_coord = prpy.util.GetPointFrom(test_array)
        numpy.testing.assert_almost_equal(numpyarray_coord, test_array)

        test_list = [0, 1, 2]
        list_coord = prpy.util.GetPointFrom(test_list)
        numpy.testing.assert_almost_equal(list_coord, test_list)

        test_tuple = (0, 1, 2)
        tuple_coord = prpy.util.GetPointFrom(test_tuple)
        numpy.testing.assert_almost_equal(tuple_coord, test_tuple)

        transform_coord = prpy.util.GetPointFrom(self.fuze_pose)
        numpy.testing.assert_almost_equal(transform_coord, self.fuze_pose[0:3, 3])

    def test_Exhibiting(self):
        self.robot.Exhibit(self.fuze)

    @nottest
    def test_Nodding(self):
        # TODO disabled until new head installed
        self.robot.NodYes()
        self.robot.NodNo()

    def test_HaltHand(self):
        self.robot.HaltHand()
        goal_pose = numpy.array([5.03348748, -1.57569674,  1.68788069,
                            2.06769058, -1.66834313,
                            1.53679821,  0.21175342], dtype='float')
        actual_pose = self.robot.GetActiveManipulator().GetDOFValues()
        numpy.testing.assert_array_almost_equal(goal_pose, actual_pose)


    def test_MiddleFinger(self):
        self.robot.MiddleFinger()

    @nottest
    def test_Wave(self):
        # TODO disabled until rmh regenerates trajectories
        self.robot.Wave()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_rogue', RogueTest)
