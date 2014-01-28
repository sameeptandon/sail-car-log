#!/usr/bin/python
# -*- coding: utf-8 -*-

import dpkt
import socket
import sys
import time


UDP_IP = '172.24.69.51'

SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

COUNTER = 0
UDPCOUNTER = 0

FILENAME = sys.argv[1]

for (ts, pkt) in dpkt.pcap.Reader(open(FILENAME, 'r')):

    COUNTER += 1
    eth = dpkt.ethernet.Ethernet(pkt)
    if eth.type != dpkt.ethernet.ETH_TYPE_IP:
        continue

    ip = eth.data
    print ip.udp.sport
    SOCK.sendto(ip.udp.data, (UDP_IP, ip.udp.sport))

    if ip.p == dpkt.ip.IP_PROTO_UDP:
        UDPCOUNTER += 1

    if COUNTER % 180 == 0:
        time.sleep(0.1)

SOCK.close()
print 'Total number of packets in the pcap file: ', COUNTER
print 'Total number of udp packets: ', UDPCOUNTER
