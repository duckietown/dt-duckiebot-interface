import socket
import struct

import time
from typing import Union

class SocketBetaflight:
    def __init__(self, addr : str, port : Union[int, str]):
        self.addr = addr
        self.port = port
        self.sudp = None

    def init(self):
        self.sudp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sudp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sudp.setblocking(False)  # Non-blocking

        return 0

    def udp_send(self, data):
        return self.sudp.sendto(data, (self.addr, self.port))

class RCPacket:
    SIMULATOR_MAX_RC_CHANNELS = 16
    
    def __init__(self):
        self._channels : list = [1000] * self.SIMULATOR_MAX_RC_CHANNELS

    def to_bytes(self):
        channel_bytes = b''.join(struct.pack('H', channel) for channel in self._channels)
        return struct.pack('d', time.time()) + channel_bytes

    @property
    def channels(self):
        """The channels of the RC packet.

        Returns:
            list: the 16 channels of the RC command.
        """
        return self._channels
    
    @channels.setter
    def channels(self, _channels: list):
        self._channels = _channels

def main():
    socketRC = SocketBetaflight('127.0.0.1', 9004, False)
    if socketRC.init():
        print("Not able to open socket")
        return

    while True:
        pkt = RCPacket()
        data = pkt.to_bytes()
        socketRC.udp_send(data)
        print(len(data))
        time.sleep(1)  # Wait for 1 second between sending packets


if __name__ == '__main__':
    main()