"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""

import herbpy, openravepy

if __name__ == "__main__":
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging();

    env, robot = herbpy.initialize(left_arm_sim=False, right_arm_sim=False,
                                   left_hand_sim=False, right_hand_sim=False,
                                   head_sim=False, segway_sim=False,
                                   moped_sim=True, attach_viewer=True)
		
