import logging
import time
import coloredlogs


class Logger(logging.Logger):
    def _none(self, *iterables):
        pass

    def __init__(self, usestdout=True, usefile=True, filename="server.log"):
        super().__init__("main")
        self.formatter = logging.Formatter(
            '[%(asctime)s][%(levelname)s][%(module)s]: %(message)s')
        st = time.time()
        self.level = logging.INFO
        self.logger = logging.getLogger("Logger")
        if (not usefile) and (not usestdout):
            self.info = self._none
            self.debug = self._none
            self.warning = self._none
            self.error = self._none
            self.critical = self._none
        if usestdout:
            coloredlogs.install(level=self.level, logger=self.logger,
                                fmt="%(asctime)s %(levelname)s [%(module)s]: %(message)s")
        if usefile:
            self.filelogger = logging.FileHandler(filename)
            self.filelogger.setLevel(self.level)
            self.filelogger.setFormatter(self.formatter)
            self.logger.addHandler(self.filelogger)
        self.logger.setLevel(self.level)
        self.info = self.logger.info
        self.debug = self.logger.debug
        self.warning = self.logger.warning
        self.error = self.logger.error
        self.critical = self.logger.critical
        self.usestdout = usestdout
        self.usefile = usefile
        self.info("Logger inited in %.3fs" % (time.time() - st))
        # logging.Logger.manager.loggerDict["main"] = self.logger
    
    def install(self, lg):
        if self.usestdout:
            coloredlogs.install(level=self.level, logger=lg,
                                fmt="%(asctime)s %(levelname)s [%(module)s]: %(message)s")
        if self.usefile:
            lg.addHandler(self.filelogger)
