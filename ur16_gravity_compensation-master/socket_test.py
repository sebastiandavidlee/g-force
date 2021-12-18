import socket

s = socket.socket()
host = "127.0.0.1"
port = 5409
s.bind((host, port))

s.listen(1)
while True:
    c, addr = s.accept()  
    print 'Got connection from', addr
    while True:
        message = raw_input(">>")
        c.sendall(message)
        try:
            print s.recv(1024)
        except:
            print "no"