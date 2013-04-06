class TrajectoryAborted(Exception):
    """
    Trajectory was aborted.
    """

class TrajectoryStalled(Exception):
    """
    Trajectory stalled.
    """

class SynchronizationException(Exception):
    """
    Controller synchronization failed.
    """
