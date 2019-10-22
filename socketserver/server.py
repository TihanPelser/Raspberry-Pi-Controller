import socket
import sys
from typing import Dict


class Server:
    def __init__(self, host: str, port: int):
        self.key = 4
        self.s: socket.socket
        self.HOST = host
        self.PORT = port

    def encrypt(self, s):
        t = ""
        s = s.decode()
        for i in s:
            t += chr(ord(i) ^ self.key)
        return str.encode(t)

    def decrypt(self, s):
        t = ""
        s = s.decode()
        for i in s:
            t += chr(ord(i) ^ self.key)
        return str.encode(t)

    def exit(self):
        print("Disconnected....")
        exit()

    def send_msg(self, msg: Dict):
        pass

    def run(self):

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.HOST, self.PORT))
            s.listen(1)
            print("Waiting for connection....")
            conn, addr = s.accept()

            with conn:
                print('Connection established at ip:{}, port:{}'.format(addr[0], self.PORT))
                print("--"*5)
                print("Waiting for client response...")
                while True:
                    data = self.decrypt(conn.recv(1024))
                    print("Client:", data.decode())
                    self.exit(data)
                    print("Server:", end="")
                    d = str.encode(input())
                    conn.sendall(self.encrypt(d))
