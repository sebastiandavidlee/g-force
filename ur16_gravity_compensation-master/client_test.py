import socket               

s = socket.socket()         
host = "127.0.0.1"         
port = 5409                 

s.connect((host, port))
while True:
    print(s.recv(1024))
    message = input(">>")
    s.send(message.encode())