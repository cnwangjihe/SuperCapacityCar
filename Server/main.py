import server
import logger

lg = logger.Logger(usefile=False)
s = server.Server(lg)
s.start()


while True:
    op = int(input("op:"))
    qos = int(input("qos:"))
    msg = input("msg:")
    s.send(op, qos, msg)
