import logging, numpy, openravepy, threading, time

logger = logging.getLogger('herbpy')

class ServoSimulator:
    def __init__(self, manip, rate, watchdog_timeout):
        self.manip = manip
        self.indices = self.manip.GetArmIndices()
        self.num_dofs = len(self.indices)
        self.q_dot = numpy.zeros(self.num_dofs)

        self.running = False
        self.watchdog = time.time()
        self.watchdog_timeout = watchdog_timeout

        self.period = 1.0 / rate
        self.timer = None
        self.mutex = threading.Lock()
        self.thread = threading.Thread(target=self.Step)
        self.thread.daemon = True
        self.thread.start()

    def SetVelocity(self, q_dot):
        q_dot_limits = self.manip.parent.GetDOFVelocityLimits(self.indices)

        if not (numpy.abs(q_dot) <= q_dot_limits).all():
            raise openravepy.openrave_exception('Desired velocity exceeds limits.')

        logger.debug('DD Acquiring')
        with self.mutex:
            if (q_dot != numpy.zeros(self.num_dofs)).any():
                self.q_dot = numpy.array(q_dot, dtype='float')
                self.watchdog = time.time()
                self.running = True
            else:
                self.running = False
        logger.debug('DD Releasing')

    def Step(self):
        while True:
            logger.debug('CC Acquiring')
            with self.mutex:
                # Copy the velocity for thread safety.
                q_dot = self.q_dot.copy()
                running = self.running

                # Stop servoing when the watchdog times out.
                now = time.time()
                if running and now - self.watchdog > self.watchdog_timeout:
                    self.q_dot = numpy.zeros(self.num_dofs)
                    self.running = False
                    logger.warning('Servo motion timed out in %.3f seconds.', now - self.watchdog)
            logger.debug('CC Releasing')

            if running:
                logger.debug('AA Acquiring')
                with self.manip.parent.GetEnv():
                    q  = self.manip.GetDOFValues()
                    q += self.period * q_dot

                    # Check joint limits.
                    logger.debug('BB Acquiring')
                    with self.manip.parent.CreateRobotStateSaver():
                        self.manip.SetActive()
                        q_min, q_max = self.manip.parent.GetActiveDOFLimits()
                    logger.debug('BB Releasing')

                    if ((q_min <= q).all() and (q <= q_max).all()):
                        self.manip.SetDOFValues(q)
                    else:
                        self.running = False 
                        logger.warning('Servo motion hit a joint limit.')
                logger.debug('AA Releasing')

            time.sleep(self.period)
