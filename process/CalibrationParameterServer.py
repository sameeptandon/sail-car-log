import SocketServer
import socket

global rx
global ry
global rz
global crx
global cry
global crz

(rx,ry,rz) = (-0.005, -0.0031, -0.03350)
(crx,cry,crz) = (0.0565,0.0201,0.0215)

global all_connected

def ParametersToString():
    global rx
    global ry
    global rz
    global crx
    global cry
    global crz

    return "%f,%f,%f,%f,%f,%f\n" % (rx,ry,rz,crx,cry,crz)

class MyUDPHandler(SocketServer.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        global all_connected
        global rx
        global ry
        global rz
        global crx
        global cry
        global crz
        data = self.request[0].strip()
        print data
        tokens = data.split(':')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if tokens[0] == 'PARAMETER_REQUEST':
            sock.sendto(ParametersToString(), (HOST, int(tokens[1])))
            all_connected.add(int(tokens[1]))
        if tokens[0] == 'PARAMETER_UPDATE':
            (rx,ry,rz,crx,cry,crz) = map(lambda x: float(x), tokens[2].split(','))
            print tokens[2]
            for address in all_connected:
                if address != int(tokens[1]):
                    sock.sendto(ParametersToString(), (HOST,address)) 

if __name__ == "__main__":
    HOST, PORT = "localhost", 2929
    server = SocketServer.UDPServer((HOST, PORT), MyUDPHandler)
    all_connected = set()
    
    server.serve_forever()
