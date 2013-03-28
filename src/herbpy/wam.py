import cbirrt, chomp, logging, openravepy
from planner import PlanningError 

def SetStiffness(manipulator, stiffness):
    try:
        manipulator.arm_controller.SendCommand('SetStiffness {0:f}'.format(stiffness))
        return True
    except openravepy.openrave_exception, e:
        logging.error(e)
        return False

def MoveHand(manipulator, preshape, timeout=None):
    if len(preshape) != 4:
        logging.error('Preshape has the wrong dimensions; expected 4, got {0:d}'.format(len(preshape)))
        return False

    manipulator.hand_controller.SetDesired(preshape)
    if timeout == None:
        manipulator.robot.WaitForController(0)
    elif timeout > 0:
        manipulator.robot.WaitForController(timeout)
    return True
