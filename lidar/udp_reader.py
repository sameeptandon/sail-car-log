#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import select
import subprocess
import sys
import re


def main():
    ifconfig_out = \
        subprocess.check_output("ifconfig eth0 | grep 'inet addr'",
                                shell=True)
    ip_addr = re.search('inet addr:(\S+)', ifconfig_out)
    if ip_addr:
        udp_ip = ip_addr.group(1)
        print udp_ip
    else:
        print 'Could not find local ip address'
        sys.exit()

    laser_port = 2368
    pos_port = 8308

    laser_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    laser_sock.bind((udp_ip, laser_port))
    pos_sock.bind((udp_ip, pos_port))

    print 'listening on ports', laser_port, 'and', pos_port
    while True:
        (inputready, _, _) = select.select([laser_sock, pos_sock], [],
                [])
        for sock in inputready:
            port = sock.getsockname()[1]
            print 'Got port: ', port
            if port == laser_port:
                (data, addr) = laser_sock.recvfrom(4096)  # Laser data is 1248 bytes
            elif port == pos_port:
                (data, addr) = laser_sock.recvfrom(4096)  # Positioning data is 234 bytes


if __name__ == '__main__':
    main()
