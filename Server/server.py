import socket
import threading
import logging
import coloredlogs
import select
import queue
import random
import zlib


class ACKPackage(object):
    def __init__(self):
        self.raw = b''
        self.crc = b''
        self.inuse = False


ACK_MAX_RETRY = 3
ACK_OVERFLOW = 0xFF
PORT = 5555
popcount_table = [0]*256
sendLock = threading.Lock()

coloredlogs.install(level=logging.DEBUG, logger=logging.getLogger(),
                    fmt="%(asctime)s %(levelname)s %(message)s")

ACKqueue = queue.Queue(0x80)
ACKok = []
ACK = []
ACKTimes = []
for _ in range(0, 0x80):
    ACKok.append(threading.Event())
    ACK.append(ACKPackage())
    ACKTimes.append(0)

online = threading.Event()

random.seed()
ids = random.randrange(0, 0x80)
status = 0
addr = []


def Getid():
    global ids
    ids = (ids+1) & 0x7F
    return ids


def popcountInit():
    global popcount_table
    for i in range(0, 0x100):
        v = i
        while v != 0:
            popcount_table[i] ^= v & 1
            v >>= 1


def popcount8(v):
    return popcount_table[v]


def popcount16(v):
    return popcount_table[v >> 8] ^ popcount_table[v & 0xFF]


def BitCount(data):
    ret = 0
    for v in data:
        ret ^= popcount8(v)
    return ret


def HammingUnpack(v):
    p = 0
    for i in range(0, 16):
        if ((1 << i) & v) != 0:
            p ^= i
    if p != 0:
        v = v ^ (1 << p)
    return v


def CRC32(data):
    cnt = len(data)
    if cnt % 4 == 0:
        cnt = 0
    else:
        cnt = 4-(cnt - int(cnt/4)*4)
    return zlib.crc32(data + b'\0'*cnt).to_bytes(4, byteorder="little")


def HammingPack(v):
    v |= (((v >> 3) ^ (v >> 5) ^ (v >> 7) ^ (v >> 9) ^
           (v >> 11) ^ (v >> 13) ^ (v >> 15)) & 1) << 1
    v |= (((v >> 3) ^ (v >> 6) ^ (v >> 7) ^ (v >> 10) ^
           (v >> 11) ^ (v >> 14) ^ (v >> 15)) & 1) << 2
    v |= (((v >> 5) ^ (v >> 6) ^ (v >> 7) ^ (v >> 12) ^
           (v >> 13) ^ (v >> 14) ^ (v >> 15)) & 1) << 4
    v |= (((v >> 9) ^ (v >> 10) ^ (v >> 11) ^ (v >> 12)
           ^ (v >> 13) ^ (v >> 14) ^ (v >> 15)) & 1) << 8
    return v


def FreeACK(Id):
    global ACKTimes, ACK
    ACKTimes[Id] = 0
    ACK[Id].inuse = False
    ACK[Id].raw = b''
    ACK[Id].crc = b''
    ACKok[Id].clear()


def ACKSingle(Id):
    global server, addr
    while True:
        if not ACKok[Id].wait(timeout=1.5):
            if ACKTimes[Id] == ACK_MAX_RETRY:
                logging.warning(f"{Id} reach ACK_MAX_RETRY time..\n")
                break
            elif ACKTimes[Id] == ACK_OVERFLOW:
                logging.warning(f"ACK array overflow!!({Id} cleaned)\n")
                break
            else:
                ACKTimes[Id] += 1
                with sendLock:
                    server.sendto(ACK[Id].raw, addr)
        else:
            break
    logging.info(f"{Id} ACKed")
    FreeACK(Id)


def SendACKPackage(Id, resend, crc):
    global sendLock, server, addr
    if Id >= (1 << 7):
        logging.error("ACK id too large")
        return
    header = (resend << 3) | (0x7 << 5) | (Id << 9)
    header = HammingPack(header)
    header = header | popcount16(header)
    raw = header.to_bytes(2, byteorder="little") + crc
    with sendLock:
        server.sendto(raw, addr)


def UDPServer():
    global addr, status, server
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('0.0.0.0', PORT))
    server.setblocking(True)
    status = 0
    logging.info("Waiting client to connect...")
    while True:
        if status:
            ready = select.select([server], [], [], 10)
            if ready[0]:
                raw, addr = server.recvfrom(0x200)
            else:
                status = 0
                logging.info("Client heart beat timeout")
                continue
        else:
            raw, addr = server.recvfrom(0x200)
            status = 1
            online.set()
            logging.info("Client online")
        rlen = len(raw)
        if rlen < 2:
            logging.error("rlen<2, drop")
            continue
        logging.debug(raw.hex())
        header = raw[0] + (raw[1] << 8)
        header = HammingUnpack(header)
        # logging.debug(f"Before unpack: {bin(header)}")
        if popcount16(header) != 0:
            logging.error("header parity check failed (>1bit err), drop")
            continue
        # logging.debug(f"After unpack: {bin(header)}")
        op = header >> 5 & 0x07
        if op == 0x7:
            Id = header >> 9
            if rlen != 6:
                logging.error("ACK len error, drop")
                continue
            crc = raw[2:6]
            logging.debug(f"Recieve CRC: {crc.hex()}")
            if (not ACK[Id].inuse) or (crc != ACK[Id].crc):
                logging.error("ACK no exist, drop")
                continue
            if (header >> 3) & 1:
                pass  # ACK request resend NOT WORKING
            ACKok[Id].set()
            continue
        cnt = (header >> 9 & 0x3F) << 3
        qos = header >> 3 & 0x01
        prt = header >> 15
        if cnt + qos + 2 != rlen:
            logging.error("len error, drop")
            continue
        if (cnt != 0) and (prt != BitCount(raw[2+qos:])):
            logging.error("content parity error, drop")
            continue
        raw = raw[2:]
        if qos:
            Id = raw[0]
            if popcount8(Id) == 1:
                logging.error("id parity check failed, drop")
                continue
            crc = CRC32(raw[1:])
            Id &= 0x7F
            SendACKPackage(Id, 0, crc)
            raw = raw[1:]
        if op == 0x0:
            logging.info(raw.decode("utf-8"))
        elif op == 0x1:
            logging.warning("Server recieve MotorStop cmd")
        elif op == 0x2:
            logging.warning("Server recieve MotorStart cmd")
        elif op == 0x3:
            logging.warning("Server recieve ChargerOff cmd")
        elif op == 0x4:
            logging.warning("Server recieve ChargerOn cmd")
        elif op == 0x5:
            if cnt != 2:
                logging.error("Recieve broken MotorSpeed")
            else:
                logging.warning(
                    f"Server recieve MotorSpeed cmd, speed: {int.from_bytes(raw,byteorder='little')}")
        elif op == 0x6:
            if cnt < 2:
                logging.error("Recieve broken PING")
            else:
                logging.info(
                    f"Recieve PING, cap voltage:{int.from_bytes(raw[0:2],byteorder='little')/4096*3.3*5}")


popcountInit()

t = threading.Thread(target=UDPServer, daemon=True)
t.start()


while True:
    if status == 0:
        online.wait()
        online.clear()
        logging.info("Input unlocked")
    op = int(input("op:"))
    qos = int(input("qos:"))
    data = b''
    cnt = 0
    if op == 0:
        while True:
            data = bytes(input("msg:"), encoding="utf-8")
            cnt = len(data)
            if cnt < 0b100000000:
                break
            logging.warning(f"msg too long (>{0b100000000-1})")
    elif op == 5:
        while True:
            v = int(input("speed:"))
            if v < 0x10000:
                break
            logging.warning("speed too large (>65535)")
        data = v.to_bytes(2, byteorder="little")
        cnt = 2
    pcnt = (((cnt & 0xF) != 0) + (cnt >> 3)) << 3
    data += b'\0' * (pcnt - cnt)
    logging.debug(f"cnt:{cnt}, pcnt:{pcnt}")
    header = (qos << 3) | (op << 5) | ((pcnt >> 3) << 9) | (BitCount(data) << 15)
    header = HammingPack(header)
    header |= popcount16(header)
    logging.debug(bin(header))
    if status == 1:
        if qos:
            Id = Getid()
            if ACK[Id].inuse:
                FreeACK(Id)
                logging.warning("ACK overflow")
            raw = header.to_bytes(2, byteorder="little") + \
                (Id | (popcount8(Id) << 7)).to_bytes(
                    1, byteorder="little") + data
            # ACK[Id].crc = CRC32(raw[3:]) # 17875c78
            ACK[Id].crc = CRC32(data)
            logging.debug(f"Mine:{ACK[Id].crc.hex()}")
            ACK[Id].raw = raw
            ACK[Id].inuse = True
            threading.Thread(target=ACKSingle, daemon=True, args=(Id,)).start()
        else:
            raw = header.to_bytes(2, byteorder="little") + data
        with sendLock:
            server.sendto(raw, addr)
    else:
        logging.error("How slowly you typed, the client has flown away")
