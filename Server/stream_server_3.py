import socket
import threading
import cv2
import numpy as np
import time


class StreamServer(threading.Thread):
    def __init__(self, logger, ip="0.0.0.0", port=5556):
        threading.Thread.__init__(self)
        self.daemon = True
        self.logger = logger
        self.ip = ip
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((self.ip, self.port))
        self.server.setblocking(True)

    def run(self):
        t = time.time()
        cnt = 0
        while True:
            raw = self.server.recv(0x20000)
            raw = np.frombuffer(raw, np.uint8)
            raw = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            cv2.imshow("image", raw)
            cv2.waitKey(1)
            cnt += 1
            x = time.time()-t
            if x > 1.0:
                t= time.time()
                self.logger.info(f"{cnt/x}fps")
                cnt = 0


if __name__ == '__main__':
    import logger
    ss = StreamServer(logger.Logger(usefile=False))
    ss.start()
    while True:
        input()
