import socket


class UdpSocket:
    def __init__(self, computer_host:str, computer_port:int):
        self.computer_host = computer_host
        self.computer_port = computer_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.computer_host,self.computer_port))
    

    def receive(self):
        data, _ = self.socket.recvfrom(65535)
        return data


    def close(self):
        self.socket.close()