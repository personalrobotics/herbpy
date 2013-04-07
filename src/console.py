"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""
import herbpy, numpy, openravepy, sys

if __name__ == "__main__":
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging();

    simulated = 'sim' in sys.argv
    viewer = 'viewer' in sys.argv

    if simulated:
        env, robot = herbpy.initialize_sim(attach_viewer=viewer)
        herbpy.logger.info('Initialized in simulation mode.')
    else:
        env, robot = herbpy.initialize_real(attach_viewer=viewer)
        herbpy.logger.info('Initialized connection with HERB.')
