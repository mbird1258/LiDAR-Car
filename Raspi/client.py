import socket
import pickle
import struct
import select

ServerIp = "192.168.0.5"

def get(blocking=True):
    # read the size of this packet
    buf = b''
    
    if not blocking:
        ready = select.select([Socket], [], [], 0)
        if not ready[0]:
            return False
    
    while len(buf) < 4:
        recv = Socket.recv(4 - len(buf))
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
        recv = Socket.recv(packetsize - len(buf))
        if not len(recv):
            break
        buf += recv
    if len(buf) != packetsize:
        return False
    
    return pickle.loads(buf)

def send(data):
    # Send Data
    data = pickle.dumps(data)
    Socket.send(struct.pack('<l', len(data)) + data)

Socket = socket.socket()
Socket.connect((ServerIp, 8000))
Socket.recv(4096)