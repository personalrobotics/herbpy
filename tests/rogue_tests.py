#!/usr/bin/env python
import roslib
import numpy, unittest
import herbpy, prpy, openravepy
from nose.tools import nottest

class RogueTest(unittest.TestCase):
    def setUp(self):
        self.env = openravepy.Environment()
        self.fuze = prpy.rave.add_object(self.env, 'fuze_bottle', 'objects/fuze_bottle.kinbody.xml')
        self.fuze_pose = numpy.eye(4)
        self.fuze_pose[0:3, 3] = [0.8, -0.3, 0.4]
        self.fuze.SetTransform(self.fuze_pose)
        self.fuze.Enable(True)

    def tearDown(self):
        self.env.Destroy()
        openravepy.RaveDestroy()

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

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_rogue', RogueTest)
