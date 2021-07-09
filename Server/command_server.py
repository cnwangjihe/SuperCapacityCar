import socket
import threading
import select
import queue
import random
import zlib


ACK_MAX_RETRY = 3
ACK_OVERFLOW = 0xFF
SEND_OK = 0
SEND_TOO_LONG = 1
SEND_TYPE_ERROR = 2
SEND_OFFLINE = 3


class ACKPackage(object):
    def __init__(self):
        self.raw = b''
        self.crc = b''
        self.inuse = False


class CommandServer(threading.Thread):
    def __init__(self, logger, ip="0.0.0.0", port=5555):
        threading.Thread.__init__(self)
        random.seed()
        self.daemon = True
        self.logger = logger
        self.ip = ip
        self.port = port
        self.popcount_table = [0]*256
        self.sendLock = threading.Lock()
        self.ACKqueue = queue.Queue(0x80)
        self.ACKok = []
        self.ACK = []
        self.ACKTimes = []
        self.online = threading.Event()
        self.ids = random.randrange(1, 0x80)
        self.status = 0
        self.addr = []
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((self.ip, self.port))
        self.server.setblocking(True)

    def popcountInit(self):
        for i in range(0, 0x100):
            v = i
            while v != 0:
                self.popcount_table[i] ^= v & 1
                v >>= 1

    def popcount8(self, v):
        return self.popcount_table[v]

    def popcount16(self, v):
        return self.popcount8(v >> 8) ^ self.popcount8(v & 0xFF)

    def BitCount(self, data):
        ret = 0
        for v in data:
            ret ^= self.popcount8(v)
        return ret

    def Init(self):
        for _ in range(0, 0x80):
            self.ACKok.append(threading.Event())
            self.ACK.append(ACKPackage())
            self.ACKTimes.append(0)
        self.popcountInit()

    def Getid(self):
        self.ids = (self.ids+1) & 0x7F
        if self.ids == 0:
            self.ids = (self.ids+1) & 0x7F
        return self.ids

    def HammingUnpack(self, v):
        p = 0
        for i in range(0, 16):
            if ((1 << i) & v) != 0:
                p ^= i
        if p != 0:
            v = v ^ (1 << p)
        return v

    def CRC32(self, data):
        cnt = len(data)
        if cnt % 4 == 0:
            cnt = 0
        else:
            cnt = 4-(cnt - int(cnt/4)*4)
        return zlib.crc32(data + b'\0'*cnt).to_bytes(4, byteorder="little")

    def HammingPack(self, v):
        v |= (((v >> 3) ^ (v >> 5) ^ (v >> 7) ^ (v >> 9) ^
               (v >> 11) ^ (v >> 13) ^ (v >> 15)) & 1) << 1
        v |= (((v >> 3) ^ (v >> 6) ^ (v >> 7) ^ (v >> 10) ^
               (v >> 11) ^ (v >> 14) ^ (v >> 15)) & 1) << 2
        v |= (((v >> 5) ^ (v >> 6) ^ (v >> 7) ^ (v >> 12) ^
               (v >> 13) ^ (v >> 14) ^ (v >> 15)) & 1) << 4
        v |= (((v >> 9) ^ (v >> 10) ^ (v >> 11) ^ (v >> 12)
               ^ (v >> 13) ^ (v >> 14) ^ (v >> 15)) & 1) << 8
        return v

    def FreeACK(self, Id):
        self.ACKTimes[Id] = 0
        self.ACK[Id].inuse = False
        self.ACK[Id].raw = b''
        self.ACK[Id].crc = b''
        self.ACKok[Id].clear()

    def ACKSingle(self, Id):
        flag = True
        while True:
            if not self.ACKok[Id].wait(timeout=1.5):
                if self.ACKTimes[Id] == ACK_MAX_RETRY:
                    self.logger.warning(f"{Id} reach ACK_MAX_RETRY time..")
                    flag = False
                    break
                elif self.ACKTimes[Id] == ACK_OVERFLOW:
                    self.logger.warning(f"ACK array overflow!!({Id} cleaned)")
                    flag = False
                    break
                else:
                    self.ACKTimes[Id] += 1
                    with self.sendLock:
                        self.server.sendto(self.ACK[Id].raw, self.addr)
            else:
                break
        if flag:
            self.logger.info(f"{Id} ACKed")
        self.FreeACK(Id)

    def SendACKPackage(self, Id, resend, crc):
        self.logger.debug("ACK sent")
        if Id >= (1 << 7):
            self.logger.error("ACK id too large")
            return
        header = (resend << 3) | (0x7 << 5) | (Id << 9)
        header = self.HammingPack(header)
        header = header | self.popcount16(header)
        raw = header.to_bytes(2, byteorder="little") + crc
        with self.sendLock:
            self.server.sendto(raw, self.addr)

    def run(self):
        self.Init()
        self.logger.info("Waiting client to connect...")
        while True:
            if self.status:
                ready = select.select([self.server], [], [], 10)
                if ready[0]:
                    raw, self.addr = self.server.recvfrom(0x200)
                else:
                    self.status = 0
                    self.logger.info("Client heart beat timeout")
                    continue
            else:
                raw, self.addr = self.server.recvfrom(0x200)
                self.status = 1
                self.online.set()
                self.logger.info("Client online")
            rlen = len(raw)
            if rlen < 2:
                self.logger.error("rlen<2, drop")
                continue
            self.logger.debug(raw.hex())
            header = raw[0] + (raw[1] << 8)
            header = self.HammingUnpack(header)
            # self.logger.debug(f"Before unpack: {bin(header)}")
            if self.popcount16(header) != 0:
                self.logger.error(
                    "header parity check failed (>1bit err), drop")
                continue
            # self.logger.debug(f"After unpack: {bin(header)}")
            op = header >> 5 & 0x07
            if op == 0x7:
                Id = header >> 9
                if rlen != 6:
                    self.logger.error("ACK len error, drop")
                    continue
                crc = raw[2:6]
                self.logger.debug(f"Receive CRC: {crc.hex()}")
                if (Id == 0):
                    self.logger("ack id is zero, drop")
                    continue
                if (not self.ACK[Id].inuse) or (crc != self.ACK[Id].crc):
                    self.logger.error("ACK no exist, drop")
                    continue
                if (header >> 3) & 1:
                    pass  # ACK request resend NOT WORKING
                self.ACKok[Id].set()
                continue
            cnt = (header >> 9 & 0x3F) << 3
            qos = header >> 3 & 0x01
            prt = header >> 15
            if cnt + qos + 2 != rlen:
                self.logger.error("len error, drop")
                continue
            if (cnt != 0) and (prt != self.BitCount(raw[2+qos:])):
                self.logger.error("content parity error, drop")
                continue
            raw = raw[2:]
            if qos:
                Id = raw[0]
                if self.popcount8(Id) == 1:
                    self.logger.error("id parity check failed, drop")
                    continue
                crc = self.CRC32(raw[1:])
                Id &= 0x7F
                if (Id == 0):
                    self.logger("package id is zero, drop")
                    continue
                self.SendACKPackage(Id, 0, crc)
                raw = raw[1:]
            if op == 0x0:
                self.logger.info(raw.decode("utf-8"))
            elif op == 0x1:
                self.logger.warning("Server receive MotorStop cmd")
            elif op == 0x2:
                self.logger.warning("Server receive MotorStart cmd")
            elif op == 0x3:
                self.logger.warning("Server receive ChargerOff cmd")
            elif op == 0x4:
                self.logger.warning("Server receive ChargerOn cmd")
            elif op == 0x5:
                if cnt != 2:
                    self.logger.error("Receive broken MotorSpeed")
                else:
                    self.logger.warning(
                        f"Server receive MotorSpeed cmd, speed: {int.from_bytes(raw,byteorder='little')}")
            elif op == 0x6:
                if cnt < 2:
                    self.logger.error("Receive broken PING")
                else:
                    self.logger.info(
                        f"Receive PING, cap voltage:{round(int.from_bytes(raw[0:2],byteorder='little')/4096*3.3*5, 3)}V")

    def send(self, op, qos, data=b""):
        if isinstance(data, str):
            data = bytes(data, "utf-8")
        elif isinstance(data, bytearray):
            data = bytes(data)
        elif op == 5 and isinstance(data, int):
            try:
                data = data.to_bytes(2, byteorder="little")
            except OverflowError:
                self.logger.error("Speed too big")
                return SEND_TYPE_ERROR
        elif not isinstance(data, bytes):
            self.logger.error("Content type error")
            return SEND_TYPE_ERROR
        cnt = len(data)
        if cnt > (1 << 9):
            self.logger.error("Content too large")
            return SEND_TOO_LONG
        pcnt = (((cnt & 0xF) != 0) + (cnt >> 3)) << 3
        data += b'\0' * (pcnt - cnt)
        self.logger.debug(f"cnt:{cnt}, pcnt:{pcnt}")
        header = (qos << 3) | (op << 5) | (
            (pcnt >> 3) << 9) | (self.BitCount(data) << 15)
        header = self.HammingPack(header)
        header |= self.popcount16(header)
        self.logger.debug(bin(header))
        if self.status == 1:
            if qos:
                Id = self.Getid()
                if self.ACK[Id].inuse:
                    self.FreeACK(Id)
                    self.logger.warning("ACK overflow")
                raw = header.to_bytes(2, byteorder="little") + \
                    (Id | (self.popcount8(Id) << 7)).to_bytes(
                        1, byteorder="little") + data
                # ACK[Id].crc = CRC32(raw[3:]) # 17875c78
                self.ACK[Id].crc = self.CRC32(data)
                self.logger.debug(f"Mine:{self.ACK[Id].crc.hex()}")
                self.ACK[Id].raw = raw
                self.ACK[Id].inuse = True
                threading.Thread(target=self.ACKSingle,
                                 daemon=True, args=(Id,)).start()
            else:
                raw = header.to_bytes(2, byteorder="little") + data
            with self.sendLock:
                self.server.sendto(raw, self.addr)
            return SEND_OK
        else:
            self.logger.error("Client offline")
            return SEND_OFFLINE
