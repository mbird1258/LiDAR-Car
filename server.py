import socket
import pickle
import struct
import select

NumOfDevices = 1

def get(blocking=True):
    data = []
    for Conn in ConnArr:
        # read the size of this packet
        buf = b''
        
        if not blocking:
            ready = select.select([Conn], [], [], 0)
            if not ready[0]:
                return False
        
        while len(buf) < 4:
            recv = Conn.recv(4 - len(buf))
            if not len(recv):
                break
            buf += recv
        if len(buf) != 4:
            # we waited for a packet but there isn't anything
            return False
        packetsize = struct.unpack('<l', buf)[0]
        
        # read the whole packet
        buf = b''
        while len(buf) < packetsize:
            recv = Conn.recv(packetsize - len(buf))
            if not len(recv):
                break
            buf += recv
        if len(buf) != packetsize:
            return False
        
        data.append(pickle.loads(buf))
    return data

def send(data):
    # Send Data
    for Conn in ConnArr:
        data = pickle.dumps(data)
        Conn.send(struct.pack('<l', len(data)) + data)

ServerIp = socket.gethostname()

Socket = socket.socket()
Socket.bind((ServerIp, 8000))
Socket.listen()

ConnArr = []
AddrArr = []

for _ in range(NumOfDevices):
    Conn, Addr = Socket.accept()
    
    ConnArr.append(Conn)
    AddrArr.append(Addr)

send("Yippee")

print(f"\n\nStart\n\n")