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
    robot.PushGrasp(cup, yaw_range=[0, numpy.pi])

    with env:
        aabb_cup = cup.ComputeAABB()
        aabb_stack = stack.ComputeAABB()
        stack_height = 2*aabb_stack.extents()[2] + (len(cups_stacked)+1)*0.03
        manip = robot.GetActiveManipulator()

    from prpy.rave import Disabled
    with Disabled(table):
        #Lift up cup
        manip.PlanToEndEffectorOffset(direction=[0, 0, 1], distance = stack_height, position_tolerance=0.1, execute=True)

    #Move cup in x-direction
    move_cup_x, x_direc = MoveCupDistAndDirec(cup, stack, 0);
    manip.PlanToEndEffectorOffset(direction=[x_direc, 0, 0], distance = move_cup_x, position_tolerance=0.1, execute=True)
    
    #Move cup in y-direction
    move_cup_y, y_direc = MoveCupDistAndDirec(cup, stack, 1);
    manip.PlanToEndEffectorOffset(direction=[0, y_direc, 0], distance = move_cup_y, position_tolerance=0.1, execute=True)

    #Move cup down
    with Disabled(stack):
        manip.PlanToEndEffectorOffset(direction=[0, 0, -1], distance = 0.02, position_tolerance=0.1, execute=True)

    #Release cup
    manip.hand.OpenHand()
    robot.Release(cup)

    with env:
        cup.SetTransform(stack.GetTransform())
    
    #Add cup to list of cups stacked
    cups_stacked.append(cup)
    return cups_stacked

def MoveCupDistAndDirec(cup, stack, direction):
    """
    @param cup The cup to move
    @param stack The stack of cups the cup is going to be placed on
    @param direction The index for the pos coordinates of cup
    """	
    import math
    with cup.GetEnv():
        aabb_cup = cup.ComputeAABB()
        aabb_stack = stack.ComputeAABB()
    move_cup_dist = aabb_cup.pos()[direction] - aabb_stack.pos()[direction]
    move_cup_direc = -1
    if move_cup_dist < 0:
        move_cup_direc = 1
        move_cup_dist = math.fabs(move_cup_dist)

    return move_cup_dist, move_cup_direc    
