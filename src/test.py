import herbpy, openravepy, numpy

def look_at_hand(robot, manipulator):
    target = manipulator.GetEndEffectorTransform()[0:3, 3]
    robot.LookAt(target, execute=True)

simulation = False
left_home_config = numpy.array([ 2.60, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ])
left_relaxed_config = numpy.array([ 0.55, -1.82,  0.35,  1.87, -2.22, -0.66, -0.98 ])
right_home_config = numpy.array([ 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00 ])
right_relaxed_config = numpy.array([ 5.73, -1.82, -0.35, 1.87, -4.06, -0.66, 0.98 ])

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
openravepy.misc.InitOpenRAVELogging();

env, robot = herbpy.initialize_sim(attach_viewer=True, right_arm_sim=False)
