#!/bin/sh

if [ $# -ne 1 ]; then
  echo "Please provide an output file name"
  exit 1
fi

sudo tcpdump -i eth0 -w "$1" udp port 2368 or udp port 8308
