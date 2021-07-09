from functools import partial
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import numpy as np
import cv2


class StreamServerPostHandler(BaseHTTPRequestHandler):
    def __init__(self, logger, *args, **kwargs):
        self.logger = logger
        super().__init__(*args, **kwargs)

    def do_POST(self):
        data = self.rfile.read(int(self.headers['content-length']))
        data = np.frombuffer(data, np.uint8)
        data = cv2.imdecode(data, cv2.IMREAD_COLOR)
        cv2.imshow("image", data)
        cv2.waitKey(1)
        self.send_response(200)


class CommandServer(threading.Thread):
    def __init__(self, logger, ip="0.0.0.0", port=5556):
        threading.Thread.__init__(self)
        self.daemon = True
        self.logger = logger
        self.ip = ip
        self.port = port
        self.handler = partial(StreamServerPostHandler, logger)
        self.server = HTTPServer((self.ip, self.port), self.handler)

    def run(self):
        self.logger.info("Stream server stared")
        self.server.serve_forever()


if __name__ == '__main__':
    import logger
    cs = CommandServer(logger.Logger(usefile=False))
    cs.start()
    while True:
        input()
