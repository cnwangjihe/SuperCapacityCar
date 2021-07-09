import tornado.log
import tornado.ioloop
import tornado.web
import tornado.websocket
import threading
import numpy as np
import cv2


class StreamServerPostHandler(tornado.web.RequestHandler):
    def initialize(self, logger):
        self.logger = logger

    def post(self):
        data = self.request.body
        data = np.frombuffer(data, np.uint8)
        data = cv2.imdecode(data, cv2.IMREAD_COLOR)
        cv2.imshow("image", data)
        cv2.waitKey(1)
        self.set_status(200)


class StreamServerWebsocketHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, logger):
        self.logger = logger

    def open(self):
        self.logger.info("WebSocket opened")
        self.set_nodelay(True)

    def on_message(self, message):
        data = message
        self.logger.info(f"len: {len(data)}")
        # data = np.frombuffer(data, np.uint8)
        # data = cv2.imdecode(data, cv2.IMREAD_COLOR)
        # cv2.imshow("image", data)
        # cv2.waitKey(1)

    def on_close(self):
        self.logger.info("WebSocket closed")


class StreamServer(threading.Thread):
    def __init__(self, logger, ip="0.0.0.0", port=5556):
        threading.Thread.__init__(self)
        self.daemon = True
        self.logger = logger
        self.ip = ip
        self.port = port
        logger.install(tornado.log.access_log)
        logger.install(tornado.log.app_log)
        logger.install(tornado.log.gen_log)

    def run(self):
        self.logger.info("Stream server stared")
        self.ioloop = tornado.ioloop.IOLoop()
        self.server = tornado.web.Application([
            (r"/upload", StreamServerWebsocketHandler, dict(logger=self.logger))
        ])
        self.server.listen(address=self.ip, port=self.port)
        self.ioloop.start()


if __name__ == '__main__':
    import logger
    ss = StreamServer(logger.Logger(usefile=False))
    ss.start()
    while True:
        input()
