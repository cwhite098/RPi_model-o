import socket
import sys
import time
import random
from struct import pack

class Comm:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = '192.168.0.8'
        self.port = 65000
        self.server_address = (self.host, self.port)
        

    def send_data(self, data):
        message = pack('3f', data)
        sock.sendto(message, server_address)
        
        time.sleep(1)
        
def main():
    
    comm = Comm()
    comm.send_data(random.random())
    
    
if __name__ == '__main__':
    main()
    