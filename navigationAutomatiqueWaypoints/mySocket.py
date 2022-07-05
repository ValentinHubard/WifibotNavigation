import socket

class MySocket:
    
    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                            socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def connect(self):
        self.sock.connect(("192.168.1.22", 15020))
        print("connecter")

    def fermer(self):
        self.sock.close()

    def sende(self, msg):
        self.sock.send(msg)

    def receive(self, EOFChar='\036'):
        self.sock.recv(1024)