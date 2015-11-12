import logging, openravepy, prpy, actions, math, numpy
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
from contextlib import contextmanager

@ActionMethod
def StackCups(robot, cup, stack, cups_stacked):
	"""
	@param robot The robot performing the stacking
	@param cup The cup that is being stacked
	@param stack The stack of cups the cup is going to be placed on
	@param cups_stacked A list of the cups that have been stacked 
	"""
	#Grasp cup
    success = actions.PushGraspCup(robot, cup, [0, numpy.pi])

    #Lift up cup
    aabb_cup = cup.ComputeAABB()
    aabb_stack = stack.ComputeAABB()
    stack_height = 2*aabb_stack.extents()[2] + (len(cups_stacked)+1)*0.03
    if success:
        success = actions.MoveObject(robot, direction=[0, 0, 1], distance = stack_height)
    
    #Move cup in x-direction
    move_cup_x, x_direc - MoveCupDistAndDirec(cup, stack, 0);
    if success:
        success = actions.MoveObject(robot, direction=[x_direc, 0, 0], distance = move_cup_x)
    
    #Move cup in y-direction
    move_cup_y, y_direc - MoveCupDistAndDirec(cup, stack, 1);
    if success:
        success = actions.MoveObject(robot, direction=[0, y_direc, 0], distance = move_cup_y)

    #Move cup down
    if success:
        stack.Enable(False)
        success = actions.MoveObject(robot, direction=[0, 0, -1], distance = 0.02)
        stack.Enable(True)

    #Release cup
    if success:
        robot.right_arm.hand.OpenHand()
        robot.Release(cup)
        cup.SetTransform(stack.GetTransform())
        cup.Enable(False)
    
    #Add cup to list of cups stacked
    cups_stacked.append(cup)
    return success, cups_stacked

    def MoveCupDistAndDirec(cup, stack, direction):
    """
    @param cup The cup to move
    @param stack The stack of cups the cup is going to be placed on
    @param direction The index for the pos coordinates of cup
    """	
    aabb_cup = cup.ComputeAABB()
    aabb_stack = stack.ComputeAABB()
    move_cup_dist = aabb_cup.pos()[direction] - aabb_stack.pos()[direction]
    direc = -1
    if move_cup_dist < 0:
        direc = 1
        move_cup_dist = math.fabs(move_cup_dist)

    return move_cup_dist, move_cup_direc    
