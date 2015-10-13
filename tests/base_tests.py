#!/usr/bin/env python
import herbpy, numpy, unittest

env, robot = herbpy.initialize(sim=True)

class BaseMovementTest(unittest.TestCase):

    def test_Forward(self):
        """
        Test commanding base to move forward
        """
        with env:
            robot.SetTransform(numpy.eye(4))
        robot.base.Forward(1.0)
        with env:
            new_pose = robot.GetTransform()
        self.assertAlmostEqual(new_pose[0,3], 1.0)

    def test_Rotate(self):
        """
        Test commanding base to rotate in place
        """
        with env:
            robot.SetTransform(numpy.eye(4))
        rotation_amount = numpy.pi/4.
        robot.base.Rotate(rotation_amount)
        with env:
            new_pose = robot.GetTransform()

        import openravepy
        aa = openravepy.axisAngleFromRotationMatrix(new_pose)
        self.assertAlmostEqual(aa[2], rotation_amount)

    def test_PlanToPose(self):
        """
        Test base planning to a goal pose
        """
        import openravepy
        base_pose = numpy.eye(4)
        base_pose[:3,:3] = openravepy.rotationMatrixFromAxisAngle([0., 0., numpy.pi/4.])
        base_pose[:2,3] = [2., 3.]
        try:
            robot.base.PlanToBasePose(base_pose, execute=True)
        except Exception as e:
            self.fail('PlanToBasePose raised unexpected exception: %s' % str(e))

        with env:
            final_pose = robot.GetTransform()
            
        # Check that we end up within grid resolution
        self.assertAlmostEqual(final_pose[0,3], 2., delta=0.05)
        self.assertAlmostEqual(final_pose[1,3], 3., delta=0.05)
        orientation = openravepy.axisAngleFromRotationMatrix(final_pose)
        self.assertAlmostEqual(orientation[2], numpy.pi/4., delta=numpy.pi/16.)
