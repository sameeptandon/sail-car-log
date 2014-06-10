# usage: python BagToGPSMarkOut.py <bag_file> 

import rosbag
import sys
import os

bag_filename = sys.argv[1]
bag_path, bag_name = os.path.split(bag_filename)
gps_outpath = bag_path
gps_outname_mark1 = bag_name[:-4] + 'mark1.out'
gps_filename_mark1 = gps_outpath + '/' + gps_outname_mark1
gps_outname_mark2 = bag_name[:-4] + 'mark2.out'
gps_filename_mark2 = gps_outpath + '/' + gps_outname_mark2

bag = rosbag.Bag(bag_filename)

f1 = open(gps_filename_mark1, 'w')
f2 = open(gps_filename_mark2, 'w')

for topic, msg, t in bag.read_messages(topics=['/novatel_port_out']):
    if "MARK1PVAA" in msg.data:
        f1.write(msg.data)

    if "MARK2PVAA" in msg.data:
        f2.write(msg.data)

f1.close()
f2.close()


