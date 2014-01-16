import dpkt, socket, sys

UDP_IP = "127.0.0.1"
UDP_PORT = 2929

sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM) # UDP
counter=0
ipcounter=0
udpcounter=0

filename=sys.argv[1]

for ts, pkt in dpkt.pcap.Reader(open(filename,'r')):

    counter+=1
    eth=dpkt.ethernet.Ethernet(pkt) 
    if eth.type!=dpkt.ethernet.ETH_TYPE_IP:
       continue

    ip=eth.data
    sock.sendto(ip.udp.data, (UDP_IP, UDP_PORT))

    if ip.p==dpkt.ip.IP_PROTO_UDP:
       udpcounter+=1

sock.close()
print "Total number of packets in the pcap file: ", counter
print "Total number of udp packets: ", udpcounter
