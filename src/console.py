"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""
import herbpy, numpy, openravepy, sys

if __name__ == "__main__":
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging();

    if 'sim' in sys.argv:
        env, robot = herbpy.initialize_sim()
        herbpy.logger.info('Initialized in simulation mode.')
    else:
        env, robot = herbpy.initialize_real()
        herbpy.logger.info('Initialized connection with HERB.')
