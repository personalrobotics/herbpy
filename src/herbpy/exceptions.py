class HerbPyException(Exception):
    """
    Generic HerbPy exception.
    """

class TrajectoryAborted(HerbPyException):
    """
    Trajectory was aborted.
    """

class TrajectoryStalled(HerbPyException):
    """
    Trajectory stalled.
    """

class SynchronizationException(HerbPyException):
    """
    Controller synchronization failed.
    """
