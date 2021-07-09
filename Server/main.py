import command_server
import logger

lg = logger.Logger(usefile=False)
s = command_server.CommandServer(lg)
s.start()


while True:
    op = int(input("op:"))
    qos = int(input("qos:"))
    msg = input("msg:")
    s.send(op, qos, msg)
