import cbirrt, chomp, logging, openravepy
from planner import PlanningError 

def SetStiffness(manipulator, stiffness):
    try:
        manipulator.arm_controller.SendCommand('SetStiffness {0:f}'.format(stiffness))
        return True
    except openravepy.openrave_exception, e:
        logging.error(e)
        return False

