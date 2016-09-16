# Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
#   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor Carnegie Mellon University,
#       nor the names of their contributors, may be used to endorse or
#       promote products derived from this software without specific prior
#       written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
#   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-
'''An example of TSR chaining for door opening
This example uses a single TSR Chain to define constraints for opening a door. The chain has either one or two elements, depending on the 'chainlength' variable
'chainlength' defines whether to use a chain of length 1 or 2
length 1 only allows rotation about the door hinge
length 2 allows rotation about the door hinge and the door handle'''

from openravepy import *
from numpy import *
from prpy.tsr.tsr import TSR, TSRChain
import time
import sys 
import numpy, prpy, herbpy, openravepy, IPython
from catkin.find_in_workspaces import find_in_workspaces

def str2num(string):
    return array([float(s) for s in string.split()])

def MakeTransform(rot,trans):
    if size(rot,0) == 9 and size(rot,1) == 1:
        tm = rot.reshape(3,3)
    elif size(rot,0) == 3 and size(rot,1) == 3:
        tm = rot
    else:
        print('rotation improperly specified');

    if size(trans,0) == 3 and size(trans,1) == 1:
        tm = bmat('tm trans')
    elif size(trans,0) == 1 and size(trans,1) == 3:
        tm = bmat('tm trans.T')
    else:
        print('translation improperly specified');
    
    lastrow = mat([0,0,0,1])
    return bmat('tm; lastrow')

def GetRot(tm):
    return mat(tm[0:3][:,0:3].T.reshape(1,9));

def GetTrans(tm):
    return mat(tm[0:3][:,3].T);

def SerializeTransform(tm):
    rot = GetRot(tm)
    trans = GetTrans(tm)
    rottrans = bmat('rot trans')
    return Serialize1DMatrix(rottrans)

def Serialize1DMatrix(arr):
    print arr
    return '%s'%(' '.join(' %.5f'%(arr[0,f]) for f in range(0,size(arr))))

def Serialize1DIntegerMatrix(arr):
    return '%s'%(' '.join(' %d'%(arr[0,f]) for f in range(0,size(arr))))

#these are standalone functions for serialization but you should really use the classes
def SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw):
   
    return '%d %s %s %s %s'%(manipindex, bodyandlink, SerializeTransform(T0_w), SerializeTransform(Tw_e), Serialize1DMatrix(Bw))

def SerializeTSRChain2(bSampleStartFromChain,bSampleGoalFromChain,bConstrainToChain,numTSRs,allTSRstring,mimicbodyname,mimicbodyjoints):
    outstring = ' TSRChain %d %d %d %d %s %s'%(bSampleStartFromChain, bSampleGoalFromChain, bConstrainToChain, numTSRs, allTSRstring, mimicbodyname)
    if size(mimicbodyjoints) != 0:
        outstring += ' %d %s '%(size(mimicbodyjoints),Serialize1DIntegerMatrix(mimicbodyjoints))

    return outstring

def WaitForController(robot):
    robot.WaitForController(0)

if __name__ == "__main__":
    import herbpy
    env, robot = herbpy.initialize(sim=True)
    env.SetViewer('interactivemarker')
    robot.right_arm.SetActive()

    # PR kitchen
    kitchen_env = find_in_workspaces(
        search_dirs=['share'],
        project='pr_ordata',
        path='data/kitchen/pr_kitchen.env.xml',
        first_match_only=True)[0]
 
    try:
        env.Load(kitchen_env)
    except:
        print 'Fail'
    walls = env.GetKinBody('walls')
    lowercabinets = env.GetKinBody('lowercabinets')
    uppercabinets = env.GetKinBody('uppercabinets')
    fridge = env.GetKinBody('refrigerator')
    microwave = env.GetKinBody('microwave')
    dishwasher = env.GetKinBody('dishwasher')

    env.Remove(walls)
    env.Remove(lowercabinets) 
    env.Remove(uppercabinets)
    env.Remove(microwave)
    env.Remove(dishwasher)
    robot.DriveTo(fridge, planning=False)

    chainlength = 2
    
    #set printing and display options
    env.SetDebugLevel(DebugLevel.Info)
    colchecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(colchecker)

    #create problem instances
    probs_cbirrt = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt,'herb')
    
    probs_cbirrt2 = RaveCreateProblem(env,'CBiRRT')
    env.LoadProblem(probs_cbirrt2,'refrigerator')

    #get info about the fridge hinge
    fridgejointind = 0

    startik = [ 3.56542649, -0.54266226, -0.64859131,  1.43396008, -0.38364328,
       -1.03981586, -1.20178858]

    robot.SetActiveDOFValues(startik)
    
    open_tsr = robot.tsrlibrary(fridge, 'open', robot.right_arm, 0, 0.25*numpy.pi)
    T0_w0 = open_tsr[1].TSRs[0].T0_w

    if True:
        Tw0_e = open_tsr[1].TSRs[0].Tw_e

        #define bounds to only allow rotation of the door about z axis up to pi     
        Bw0 = mat([0, 0,   0, 0,   0, 0,   0, 0,  0, 0,   0, pi])
        #Bw0 = open_tsr[0].TSRs[0].Bw

        #get the second TSR's offset from the handle to the hand
        Tw1_e = open_tsr[1].TSRs[1].Tw_e
            
        #define bounds to allow rotation of the hand about z axis of the handle from -pi/2 to 0
        Bw1 = mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -0.1, 0.1])
        #Bw1 = open_tsr[0].TSRs[1].Bw

        #serialize the TSRs
        from prpy.planning.cbirrt import SerializeTSRChain
        TSRChainString = 'TSRChain %s' % SerializeTSRChain(open_tsr[1])

    #keep track of how many DOF of the mimic body are being mimiced
    numTSRChainMimicDOF = 1;

    #get goal transform
    T0_RH2 = open_tsr[0].sample()

    goalik = robot.right_arm.FindIKSolution(T0_RH2, 0)
    robot.SetActiveDOFValues(goalik)

    #set robot to starting configuration
    robot.SetActiveDOFValues(startik)

    #call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
    mimicvals = mat(zeros([1,numTSRChainMimicDOF]))
    goalik = mat(goalik)
    goaljoints = bmat('goalik  mimicvals')


    from prpy.planning import CBiRRTPlanner
    cbirrt_planner = CBiRRTPlanner(
        timelimit=1., robot_checker_factory=robot.robot_checker_factory)

    fridge.SetActiveDOFs([0])
    path = cbirrt_planner.PlanToTSR(robot, [open_tsr[1]], jointgoals=goaljoints.tolist(),
                                       save_mimic_trajectories=True)
        
    args_str = 'RunCBiRRT smoothingitrs 50 jointgoals %d %s %s'%(size(goaljoints),Serialize1DMatrix(goaljoints), TSRChainString)

    door_controller = openravepy.RaveCreateController(robot.GetEnv(), 'IdealController')
    fridge.SetController(door_controller)
    

    fpath = cbirrt_planner.GetMimicPath(fridge.GetName(), env=fridge.GetEnv())

    
    fcspec = fpath.GetConfigurationSpecification()
    fcspec.AddDeltaTimeGroup()
    for idx in range(fpath.GetNumWaypoints()):
        wpt = fpath.GetWaypoint(idx)
        # TODO: This fails right now bc waypoint is len 1 and should be len 2
        fcspec.InsertDeltaTime(wpt, 0.1)
        fpath.Insert(idx, wpt, True)

    fridge.GetController().SetPath(fpath)
    traj = robot.ExecutePath(path)

    import IPython; IPython.embed()

    print "Press return to exit."
    sys.stdin.readline()

