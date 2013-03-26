import herbpy, openravepy

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
openravepy.misc.InitOpenRAVELogging();

env, robot = herbpy.initialize(left_arm_sim=False, right_arm_sim=False,
                               left_hand_sim=False, right_hand_sim=False,
                               head_sim=False, segway_sim=False,
                               moped_sim=True, attach_viewer=True)

def look_at_hand(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    herbpy.look_at(robot, target, execute=True)
