#!/usr/bin/env python
import herbpy, unittest

class IntegrationTest(unittest.TestCase):

    def test_initialize(self):
        try:
            env, robot = herbpy.initialize(sim=True)
        except Exception as e:
            self.fail("Exception thrown on herbpy initialize: %s" % str(e))

    def test_herb_components(self):
        env, robot = herbpy.initialize(sim=True)

        self.assertIsNotNone(robot.head)
        self.assertIsNotNone(robot.right_arm)
        self.assertIsNotNone(robot.left_arm)
        self.assertIsNotNone(robot.right_hand)
        self.assertIsNotNone(robot.left_hand)

