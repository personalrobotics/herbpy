#!/usr/bin/env python
import roslib
import numpy, unittest
import herbpy, prpy

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
        kinbody_coord = herbpy.action.GetPointFrom(self.env, self.fuze)
        numpy.testing.assert_almost_equal(kinbody_coord, self.fuze_pose[0:3, 3])

        test_array = numpy.array([0, 1, 2])
        numpyarray_coord = herbpy.action.GetPointFrom(self.env, test_array)
        numpy.testing.assert_almost_equal(numpyarray_coord, test_array)

        test_list = [0, 1, 2]
        list_coord = herbpy.action.GetPointFrom(self.env, test_list)
        numpy.testing.assert_almost_equal(list_coord, test_list)

        test_tuple = (0, 1, 2)
        tuple_coord = herbpy.action.GetPointFrom(self.env, test_tuple)
        numpy.testing.assert_almost_equal(tuple_coord, test_tuple)

        transform_coord = herbpy.action.GetPointFrom(self.env, self.fuze_pose)
        numpy.testing.assert_almost_equal(transform_coord, self.fuze_pose[0:3, 3])

    def test_Exhibiting(self):
        self.robot.Exhibit(self.fuze)
    
    def test_Nodding(self):
        self.robot.NodYes()
        self.robot.NodNo()

    def test_HaltHand(self):
        self.robot.HaltHand()

    def test_MiddleFinger(self):
        self.robot.MiddleFinger()

    def test_Wave(self):
        self.robot.Wave()

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_rogue', RogueTest)
