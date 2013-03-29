import cbirrt, chomp, logging, openravepy
from planner import PlanningError 
from methodlist_decorator import CreateMethodListDecorator

WamMethod = CreateMethodListDecorator()

@WamMethod
def SetStiffness(manipulator, stiffness):
    try:
        manipulator.arm_controller.SendCommand('SetStiffness {0:f}'.format(stiffness))
        return True
    except openravepy.openrave_exception, e:
        logging.error(e)
        return False

@WamMethod
def MoveHand(manipulator, f1=None, f2=None, f3=None, spread=None, timeout=None):
    # Default any None's to the current DOF values.
    hand_indices = sorted(manipulator.GetChildDOFIndices())
    preshape = manipulator.parent.GetDOFValues(hand_indices)

    if f1     is not None: preshape[0] = f1
    if f2     is not None: preshape[1] = f2
    if f3     is not None: preshape[2] = f3
    if spread is not None: preshape[3] = spread

    manipulator.hand_controller.SetDesired(preshape)
    if timeout == None:
        manipulator.parent.WaitForController(0)
    elif timeout > 0:
        manipulator.parent.WaitForController(timeout)
    return True
