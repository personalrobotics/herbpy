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
    import offscreen_render
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
        robot.PlanToTSR(point_tsr, execute=True, 
              ranker=Naturalness(focus_trans, goal_name))
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
def Exhibit(robot, obj, manip=None, distance=0.1, wait=2, render=True):
    """
    @param robot The robot performing the exhibit
    @param obj The object being exhibited
    @param manip The maniplator to perform the exhibit
    @param distance The distance the object will be lifted up
    @param wait The amount of time the object will be held up in seconds
    @param render Render tsr samples during planning
    """

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

    #TODO: possibly then release, openhand, retract. 

@ActionMethod
def Nod(robot, word='yes'):
    """
    @param robot The robot being used to nod
    @param word Shakes up and down for 'yes' and left and right for 'no'
    """
    before = robot.head.GetDOFValues()

    if word == 'yes':
        first = numpy.array([ 0., -0.43588399]) #Down
        second = numpy.array([ 0.,  0.26307139]) #Up
    elif word == 'no':
        first = numpy.array([-0.45916359,  0.]) #Left 
        second = numpy.array([0.45916359,  0.]) #Right
    else:
        raise prpy.exceptions.PrPyException('Word Not Recognized')

    robot.head.MoveTo(first)
    robot.head.MoveTo(second)
    robot.head.MoveTo(before)

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
    Stop(robot, manip)

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

class Naturalness(object):
    def __init__(self, focus_trans, goal_name):
        self.focus_trans = focus_trans
        self.goal_name = goal_name

        #Set up sensor parameters
        self.sensor_length = 640
        self.sensor_width = 480
        self.sensor_weight = 255
        self.scoreMask = self.weightedScoreArray()
       
        feature_path = FindCatkinResource('herbpy', 
                'config/natural_feature_weights.pickle')
         
        try:
            self.featureWeights = cPickle.load(open(feature_path, 'rb'))
        except IOError as e:
            raise ValueError('Failed loading pointing weights \
                    from "{:s}".'.format(feature_path))
        
        
    def __call__(self, robot, ik_solutions):
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.manip = robot.GetActiveManipulator()
        num_sols = ik_solutions.shape[0]
        results = numpy.zeros(num_sols)

        #Create sensor
        self.sensor = openravepy.RaveCreateSensor(self.env,
                'offscreen_render_camera')
        self.sensor.SendCommand('setintrinsic 529 525 328 267 0.01 10')
        self.sensor.SendCommand('setdims '+str(self.sensor_length)+
                ' '+str(self.sensor_width))
        self.sensor.Configure(openravepy.Sensor.ConfigureCommand.PowerOn)
        resetDOFs = self.manip.GetDOFValues()             
 
        #Score each configuration
        for i in xrange(0, num_sols):
            results[i] = self.score_pose(ik_solutions[i])

        self.manip.SetDOFValues(resetDOFs)
        return results

    def score_pose(self, ik_sol):
        """Set the robot in the configuration and then total up, 
        for each scoring parameter, the weight of that parameter
        times its score"""
        self.manip.SetDOFValues(ik_sol)
        self.pose = self.manip.GetEndEffectorTransform()

        total = 0
        total += self.compute_score('/right/j1', 'ShoulderRotation', 1.5, 2, 0) 
        total += self.compute_score('/right/j5', 'ElbowRotation', -0.5, 2, 1.5)
        total += self.compute_score('/right/j6', 'WristAngle', 0, 1, 0)
        total += self.compute_score('/right/j7', 'WristRotation', 0, 1, 0)
        total += self.wristOffset('WristRelative')
        total += self.objectDistance('Distance')

        #If the focus is a point in space, not an object, there
        #is no concept of occulsion
        if self.goal_name is not None:
            total += self.score_occulsion()
        return total

    def compute_score(self, joint, dict_name, optimal, full_range, offset):
        """For the generic joint based parameter, score based on 
        who far the configuration differs from the 'optimal' """
        actual = self.robot.GetJoint(joint).GetValue(1)+(offset*numpy.pi)
        optimal_val = (optimal+offset)*numpy.pi
        full_range_val = full_range*numpy.pi
        weight = self.featureWeights[dict_name]
        return (abs(optimal_val - actual) / full_range_val)*weight

    def wristOffset(self, dict_name):
        """For finding how far the rotation of the wrist is offset
        from being horizontal, find the angle between z value of the
        wrist pose and the j unit vector. Then score by how much it
        differs from the optimal, being horizontal."""
        z = self.pose[0:3, 2]
        z_length = numpy.sqrt(z[0]**2 + z[1]**2 + z[2]**2)
        angle = (numpy.arccos(z[1] / z_length)) / numpy.pi
        optimal = 0
        full_range = 2
        offset = abs(optimal - angle) / full_range
        return self.featureWeights[dict_name] * offset 

    def objectDistance(self, dict_name):
        """Compute the distance between the end effector and the object."""
        [px, py, pz] = self.pose[0:3, 3]
        [gx, gy, gz] = self.focus_trans[0:3, 3]
        dist = numpy.sqrt((px-gx)**2 + (py - gy)**2 + (pz - gz)**2)
        return self.featureWeights[dict_name]*dist

    def weightedScoreArray(self):
        """The occulsion value is scored against a weighted array that
        weights items closer to the center, i.e near the point's focus
        higher then the periphery."""
        l = (self.sensor_length - 1) / 2
        w = (self.sensor_width - 1) /2
        w = numpy.fromfunction(lambda i, j: 1 / (1+numpy.sqrt((l-i)**2 
            +(w-j)**2)), (self.sensor_length, self.sensor_width), dtype=int)
        return numpy.ravel(w)

    def valFromPhoto(self, img):
        """Takes the image generated by sensor and computes
        the weighted score."""
        data = img.imagedata
        compact_array = numpy.apply_over_axes(numpy.sum, data, [2])
        reshape_array = numpy.ravel(compact_array)
        divide_out = reshape_array / self.sensor_weight
        goal = numpy.dot(self.scoreMask, divide_out)
        return goal

    def score_occulsion(self):
        """Sensor captures base photo with just the goal object
        and scores it against a photo with all objects in the scene
        overlaid on the base photo."""
        pose = self.manip.GetEndEffectorTransform()
        self.sensor.SetTransform(pose)
       
        self.sensor.SendCommand('addbody '+self.goal_name+' '+
                str(self.sensor_weight)+' 0 0')
        self.sensor.SimulationStep(0.01)
        data_solo = self.sensor.GetSensorData()
        solo_goal = self.valFromPhoto(data_solo)
       
        #Get all other objects in the scene and add them to the image
        allObjs = self.env.GetBodies()
        for j in allObjs:
            name = j.GetName()
            if ((name != self.goal_name) and (name != self.robot.GetName())):
                self.sensor.SendCommand('addbody '+name+' 0 0 0 0')

        self.sensor.SimulationStep(0.01)
        data_all = self.sensor.GetSensorData()
        all_goal = self.valFromPhoto(data_all)
 
        if solo_goal == 0:
            score = 0
        else:
            score = (float(solo_goal - all_goal) / float(solo_goal))

        self.sensor.SendCommand('clearbodies')
        return score*self.featureWeights['OcculsionScore'] 
