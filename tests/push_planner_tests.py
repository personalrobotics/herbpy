#!/usr/bin/enev python
import unittest
'''
class PushPlannerTest(unittest.TestCase):
    def setUp(self):

        import herbpy, numpy, openravepy, os

        # Environment setup
        self.env, self.robot = herbpy.initialize(sim=True)

        # Table
        table_path = os.path.join('objects', 'table.kinbody.xml')
        self.table = self.env.ReadKinBodyXMLFile(table_path)
        table_transform = numpy.eye(4)
        table_transform[:3,:3] = openravepy.rotationMatrixFromAxisAngle([1.20919958, 1.20919958, 1.20919958])
        table_transform[:3,3] = [0.65, 0.0, 0.0]
        self.table.SetTransform(table_transform)
        self.env.Add(self.table)

        # Glass
        glass_path = os.path.join('objects', 'plastic_glass.kinbody.xml')
        self.glass = self.env.ReadKinBodyXMLFile(glass_path)
        self.env.Add(self.glass)
        glass_transform = numpy.eye(4)
        glass_transform[:3,3] = [0.6239455840637041, -0.4013916328109689, 0.75]
        self.glass.SetTransform(glass_transform)

        # Planning setup

        # Goal pose for the object
        goal_in_table = [0.15, 0., -0.03, 1.]
        goal_in_world = numpy.dot(self.table.GetTransform(), goal_in_table)
        self.goal_pose = goal_in_world[:2]
        self.goal_radius = 0.1    

        # Manipulators and hand shape
        self.push_arm = self.robot.right_arm
        self.push_arm.hand.MoveHand(spread=0, f1=0.75, f2=0.75, f3=0.75)

    def tearDown(self):
        self.env.Destroy()

    def test_Pushing(self):
        self.push_arm.SetActive()
        start_pose = [ 4.49119545, -1.59899798, -0.6,  1.65274406, -1.7742985,  -0.63854765, -1.23051631]
        self.push_arm.PlanToConfiguration(start_pose, execute=True)
        
        # Plan to push the object
        from herbpy.action.pushing import PushToPoseOnTable
        traj = self.robot.PushToPoseOnTable(obj=self.glass, 
                                            table=self.table,
                                            goal_position=self.goal_pose,
                                            goal_radius=self.goal_radius,
                                            manip=self.push_arm,
                                            max_plan_duration=300,
                                            num_control_samples=1,
                                            render=False)
'''
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_push_planner', PushPlannerTest)
