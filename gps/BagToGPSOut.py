# usage: python BagToGPSOut.py <bag_file> 

import rosbag
import sys
import os

bag_filename = sys.argv[1]
bag_path, bag_name = os.path.split(bag_filename)
gps_outpath = bag_path
gps_outname = bag_name[:-4] + '.out'
gps_filename = gps_outpath + '/' + gps_outname

bag = rosbag.Bag(bag_filename)

f = open(gps_filename, 'w')

for topic, msg, t in bag.read_messages(topics=['/novatel_port_out']):
    if "#MARK1PVAA" in msg.data:
        f.write(msg.data)

f.close()


