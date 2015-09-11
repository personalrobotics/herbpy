import logging, openravepy, prpy 
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
from contextlib import contextmanager 
from prpy.util import FindCatkinResource
import numpy, cPickle, time, os.path

logger = logging.getLogger('herbpy')

@ActionMethod
def Point(robot, focus, manip=None, render=False):
    """
    @param robot The robot performing the point
    @param focus The 3-D coordinate in space or object 
                 that is being pointed at
    @param manip The manipulator to perform the point with. 
                 This must be the right arm
    @param render Render tsr samples during planning
    """

    #Pointing at an object
    if type(focus) == openravepy.openravepy_int.KinBody:
        focus_trans = focus.GetTransform()
        goal_name = focus.GetName()
    #Pointing at a point in space
    elif (type(focus) == numpy.ndarray) and (focus.ndim == 1):
        if len(focus) != 3:
            raise prpy.exceptions.PrPyExceptions('A point in space must \
                    contain exactly three coordinates')
        focus_trans = numpy.eye(4)
        focus_trans[0:3, 3] = focus
        goal_name = None
    #Possible feature: if many objects then pass to multipoint
    else:
        raise prpy.exceptions.PrPyException('Focus of the point is an \
                unknown object')

    if manip is None:
        print "Setting the right arm to be the active manipulator."
        manip = robot.right_arm
        manip.SetActive()

    if manip.GetName() != 'right':
        raise prpy.exceptions.PrPyException('Pointing is only defined \
                on the right arm.')

    point_tsr = robot.tsrlibrary(None, 'point', focus_trans, manip)

    with prpy.viz.RenderTSRList(point_tsr, robot.GetEnv(), render=render):
        robot.PlanToTSR(point_tsr, execute=True)
    robot.right_hand.MoveHand(f1=2.4, f2=0.8, f3=2.4, spread=3.14)

@ActionMethod
def Present(robot, focus, manip=None, render=True):
    """
    @param robot The robot performing the presentation
    @param focus The 3-D coordinate in space or object that 
                 is being presented
    @param manip The manipulator to perform the presentation with. 
                 This must be the right arm.
    @param render Render tsr samples during planning
    """
    #Presenting an object
    if type(focus) == openravepy.openravepy_int.KinBody:
        focus_trans = focus.GetTransform()
    #Presenting a point in space
    elif (type(focus) == numpy.ndarray) and (focus.ndim == 1):
        if len(focus) != 3:
            raise prpy.exceptions.PrPyExceptions('A presentation in space \
                    must contain exactly three coordinates')
        focus_trans = numpy.eye(4)
        focus_trans[0:3, 3] = focus
    else:
        raise prpy.exceptions.PrPyException('Focus of the presentation is an \
                unknown object')

    if manip is None:
        print "Setting the right arm to be the active manipulator."
        manip = robot.right_arm
        manip.SetActive()

    if manip.GetName() != 'right':
        raise prpy.exceptions.PrPyException('Presenting is only defined \
                on the right arm.')

    present_tsr = robot.tsrlibrary(None, 'present', focus_trans, manip)
    
    with prpy.viz.RenderTSRList(present_tsr, robot.GetEnv(), render=render):
        robot.PlanToTSR(present_tsr, execute=True)
    
    robot.right_hand.MoveHand(f1=1, f2=1, f3=1, spread=3.14)

@ActionMethod
def Sweep(robot, start, end, manip=None, margin=0.3, render=True):
    """
    @param robot The robot performing the sweep
    @param start The object or 3-d position that marks the start
    @param end The object of 3-d position that marks the end
    @param manip The manipulator to perform the sweep
    @param margin The distance between the start object and the hand,
                  so the vertical space between the hand and objects. 
                  This must be enough to clear the objects themselves.
    @param render Render tsr samples during planning
    """

    if type(start) == openravepy.openravepy_int.KinBody: 
        start_trans = start.GetTransform()
        start_coords = start_trans[0:3, 3]
    elif (type(start) == numpy.ndarray) and (start.ndim == 1):
        if len(start) != 3:
            raise prpy.exceptions.PrPyExceptions('A point in space must \
                    contain exactly three coordinates')
        start_coords = start
    else: 
        raise prpy.exceptions.PrPyException('Focus of the point is an \
                unknown object')

    if type(end) == openravepy.openravepy_int.KinBody:
        end_trans = end.GetTransform()
        end_coords = end_trans[0:3, 3]
    elif (type(end) == numpy.ndarray) and (end.ndim == 1):
        if len(end) != 3:
            raise prpy.exceptions.PrPyExceptions('A point in space must \
                    contain exactly three coordinates')
        end_coords = end
        end_trans = numpy.eye(4)
        end_trans[0:3, 3] = end_coords
    else:
        raise prpy.exceptions.PrPyException('Focus of the point is an \
                unknown object')

    if manip is None:
        manip = robot.GetActiveManipulator()

    #ee_offset : such that the hand, not wrist, is above the object
    #hand_pose : places the hand above the start location
    if manip.GetName() == 'right':
        hand = robot.right_hand
        ee_offset = -0.2
        hand_pose = numpy.array([[ 0, -1, 0, start_coords[0]],
                                 [ 0,  0, 1, (start_coords[1]+ee_offset)],
                                 [-1,  0, 0, (start_coords[2]+margin)],
                                 [ 0,  0, 0, 1]])

    elif manip.GetName() == 'left':
        hand = robot.left_hand
        ee_offset = 0.2
        hand_pose = numpy.array([[ 0,  1, 0, start_coords[0]],
                                 [ 0,  0, -1, (start_coords[1]+ee_offset)],
                                 [-1,  0, 0, (start_coords[2]+margin)],
                                 [ 0,  0, 0, 1]])
  
    else:
        raise prpy.exceptions.PrPyException('Manipulator does not have an \
                 associated hand')

    hand.MoveHand(f1=1, f2=1, f3=1, spread=3.14)
    manip.PlanToEndEffectorPose(hand_pose)

    #TSR to sweep to end position
    sweep_tsr = robot.tsrlibrary(None, 'sweep', end_trans, manip)

    with prpy.viz.RenderTSRList(sweep_tsr, robot.GetEnv(), render=render):
        robot.PlanToTSR(sweep_tsr, execute=True)

@ActionMethod
def Exhibit(robot, obj, manip=None, distance=0.1, wait=2, release=True, render=True):
    """
    @param robot The robot performing the exhibit
    @param obj The object being exhibited
    @param manip The maniplator to perform the exhibit
    @param distance The distance the object will be lifted up
    @param wait The amount of time the object will be held up in seconds
    @param render Render tsr samples during planning
    """

    if manip is None:
        manip = robot.GetActiveManipulator()

    preconfig = manip.GetDOFValues()
    robot.Grasp(obj)

    #Lift the object - write more tsrs
    lift_tsr = robot.tsrlibrary(obj, 'lift', manip, distance=distance)
    
    with prpy.viz.RenderTSRList(lift_tsr, robot.GetEnv(), render=render):
        robot.PlanToTSR(lift_tsr, execute=True)

    #Wait for 'time'
    time.sleep(wait)

    #'Unlift' the object, so place it back down
    unlift_tsr = robot.tsrlibrary(obj, 'lift', manip, distance=-distance)
    
    with prpy.viz.RenderTSRList(unlift_tsr, robot.GetEnv(), render=render):
        robot.PlanToTSR(unlift_tsr, execute=True)

    if release:
        robot.Release(obj)
        manip.hand.OpenHand()
        manip.PlanToConfiguration(preconfig)

@ActionMethod
def Nod(robot, word='yes', inc=4):
    """
    @param robot The robot being used to nod
    @param word Shakes up and down for 'yes' and left and right for 'no'
    """

    import time
    pause = 0.15

    if word == 'no':
        for i in xrange(inc):
            robot.head.Servo([ 1, 0])
            time.sleep(pause)
        for i in xrange((inc*3)):
            robot.head.Servo([-1, 0])
            time.sleep(pause)
        for i in xrange((inc*2)):
            robot.head.Servo([ 1, 0])
            time.sleep(pause)
    elif word == 'yes':
        for i in xrange(inc):
            robot.head.Servo([ 0,  1])
            time.sleep(pause)
        for i in xrange((inc*3)):
            robot.head.Servo([ 0, -1])
            time.sleep(pause)
        for i in xrange((inc*2)):
            robot.head.Servo([ 0,  1])
            time.sleep(pause)
    else:
        raise prpy.exceptions.PrPyException('Word Not Recognized')


@ActionMethod
def HaltHand(robot, manip=None):
    """
    @param robot The robot being used for the stopping gesture
    @param manip The manipulator being used for the stopping gesture
    """

    if manip == None:
        manip = robot.GetActiveManipulator()

    if manip.GetName() == 'right':
        pose = numpy.array([5.03348748, -1.57569674,  1.68788069,
                            2.06769058, -1.66834313,
                            1.53679821,  0.21175342])
        
        manip.PlanToConfiguration(pose)
        robot.right_hand.MoveHand(f1=0, f2=0, f3=0, spread=3.14)
    elif manip.GetName() == 'left':
        pose = numpy.array([ 1.30614268, -1.76      , -1.57063853,
                             2.07228362,  1.23918377,
                             1.46215605, -0.12918424])

        manip.PlanToConfiguration(pose)
        robot.left_hand.MoveHand(f1=0, f2=0, f3=0, spread=3.14)
    else: 
        raise prpy.exceptions.PrPyException('Stop is only defined \
                                                for the left and right arm.')

@ActionMethod
def HighFive(robot, manip=None, wait=7):
    """
    @param robot The robot being used to high five
    @param manip The manipulator being used to high five
    @param wait The time to hold the hand up after feeling the 
                force before the hand retracts
    """
    if manip is None:
        manip = robot.GetActiveManipulator()
        
    if manip.GetName() == 'left':
        hand = robot.left_hand
    elif manip.GetName() == 'right':
        hand = robot.right_hand
    else:
        prpy.exceptions.PrPyException('High Fiving is only defined \
                for the left and right arm')

    before = manip.GetDOFValues()
    hand.TareForceTorqueSensor()

    #Move into canonical high fiving position
    HaltHand(robot, manip)

    #TODO wait in till robot.right_hand.GetForceTorque() reads something

    #After having felt force, wait a few seconds 
    time.sleep(wait)

    #Retract the arm to where it was before the interaction
    manip.PlanToConfiguration(before)

@ActionMethod
def MiddleFinger(robot, manip=None):
    """
    @param robot The robot being used to give the middle finger
    @param manip The manipulator being used to give the middle finger.
                 Must be either the right or left arm.
    """
    if manip == None:
        manip = robot.GetActiveManipulator()

    if manip.GetName() == 'right':
        right_dof = numpy.array([ 5.03348748, -1.57569674,  1.68788069,  
                                  2.06769058, -1.66834313,
                                  1.53679821,  0.21175342])
         
        manip.PlanToConfiguration(right_dof, execute=True)
        robot.right_hand.MoveHand(f1=2, f2=2, f3=0, spread=3.14)

    elif manip.GetName() == 'left':
        left_dof = numpy.array([ 1.30614268, -1.76      , -1.57063853,  
                                 2.07228362,  1.23918377,
                                 1.46215605, -0.12918424])

        manip.PlanToConfiguration(left_dof, execute=True)
        robot.left_hand.MoveHand(f1=2, f2=2, f3=0, spread=3.14)
    else: 
        raise prpy.exceptions.PrPyException('The middle finger is only defined \
                                for the left and right arm.')

@ActionMethod
def Wave(robot):
    """
    @param robot The robot waving with their right arm
    """

    from prpy.rave import load_trajectory
    from prpy.util import FindCatkinResource
    env = robot.GetEnv()

    manip = robot.right_arm
    manip.SetActive()
    print "Setting right arm as the active manipulator"

    wave_path = FindCatkinResource('herbpy', 'config/waveTrajs/')
    try:
        traj0 = load_trajectory(env, wave_path+'wave0.xml')
        traj1 = load_trajectory(env, wave_path+'wave1.xml')
        traj2 = load_trajectory(env, wave_path+'wave2.xml')
        traj3 = load_trajectory(env, wave_path+'wave3.xml')
    except IOError as e:
        raise ValueError('Failed loading wave trajectory from "{:s}".'.format(
            wave_path))

    robot.HaltHand(manip=manip)
    robot.ExecuteTrajectory(traj0)
    robot.ExecuteTrajectory(traj1)
    robot.ExecuteTrajectory(traj2)
    robot.ExecuteTrajectory(traj3)
