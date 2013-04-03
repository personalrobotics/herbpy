"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""
import herbpy, openravepy

if __name__ == "__main__":
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    openravepy.misc.InitOpenRAVELogging();
    env, robot = herbpy.initialize_real()
