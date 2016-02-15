import prpy, numpy
from prpy.action import ActionMethod

@ActionMethod
def StackCups(robot, table, cup, stack, cups_stacked):
    """
    @param robot The robot performing the stacking
    @param cup The cup that is being stacked
    @param stack The stack of cups the cup is going to be placed on
    @param cups_stacked A list of the cups that have been stacked 
    """
	#Grasp cup
    env = robot.GetEnv()

    with env:
        aabb_cup = cup.ComputeAABB()
        aabb_stack = stack.ComputeAABB()
        stack_height = 2*aabb_stack.extents()[2] + (len(cups_stacked)+1)*0.03
        manip = robot.GetActiveManipulator()
        herb_in_world = robot.GetTransform()
        cup_in_world = cup.GetTransform()

    cup_in_herb = numpy.dot(numpy.linalg.inv(herb_in_world), cup_in_world)
    theta = numpy.arctan2(cup_in_herb[1,0], cup_in_herb[0,0])

    if manip == robot.left_arm:    
        yaw_in_herb = numpy.array([0, numpy.pi])
    else:
        yaw_in_herb = numpy.array([-numpy.pi, 0])   
    
    yaw_in_cup = yaw_in_herb - theta

    robot.PushGrasp(cup, yaw_range=yaw_in_cup)

    from prpy.rave import Disabled
    with Disabled(table):
        #Lift up cup
        manip.PlanToEndEffectorOffset(direction=[0, 0, 1], distance = stack_height, position_tolerance=0.1, execute=True)

    with env:
        aabb_cup = cup.ComputeAABB()
        aabb_stack = stack.ComputeAABB()
    direc = numpy.array([0., 0., 0.])    
    direc[:2] = aabb_stack.pos()[:2] - aabb_cup.pos()[:2]
    dist = numpy.linalg.norm(direc)
    print aabb_cup.pos(), aabb_stack.pos(), direc, dist
    manip.PlanToEndEffectorOffset(direction=direc, distance = dist, position_tolerance=0.05, execute=True)

    #Move cup down
    with Disabled(stack):
        manip.PlanToEndEffectorOffset(direction=[0, 0, -1], distance = 0.02, position_tolerance=0.1, execute=True)

    #Release cup
    manip.hand.OpenHand()

    with env:
        robot.Release(cup)
        cup.SetTransform(stack.GetTransform())
    
    #Add cup to list of cups stacked
    cups_stacked.append(cup)
    return cups_stacked
  
