#!/usr/bin/python
# -*- coding: utf-8 -*-

import bisect
import os
import sys
EXPORT_RDR = True
try:
    import rosbag
except ImportError, e:
    print 'Failed to import rosbag, not exporting radar'
    EXPORT_RDR = False

sys.path.append('../process')
from GPSReader import GPSReader


class FrameFinder:

    """ Creates a mapping from cloud to frames.
    """

    frame_to_cloud_map = {}
    map_file_path = ""
    map_file_name = ""

    def __init__(self, gps_file, frames_folder, radar_bag_file, write_to_file):
        if frames_folder[-1] == '/':
            frames_folder = frames_folder[0:-1]
        
        basename = frames_folder.replace('_frames', '')
        self.map_file_name = basename + ".map"
        
        reader = GPSReader(gps_file)

        # Camera time should already be sorted because frame # always increases
        camera_times = [utc_from_gps(data['week'], data['seconds'])
                        for data in reader.getData()]

        ldr_times = \
            [long(os.path.splitext(os.path.basename(ldr_file))[0])
             for ldr_file in os.listdir(frames_folder)
             if ldr_file.endswith('.ldr')]

        # Need to sort the ldr_times because listdir returns undefined ordering
        ldr_times.sort()

        if EXPORT_RDR:
            rdr_times = unpack_bag(basename, radar_bag_file)

        for frame_number, camera_time in enumerate(camera_times):
            # Find the closest time in ldr times
            nearest_index_ldr = bisect.bisect(ldr_times, camera_time)
            nearest_index_rdr = -1
            if EXPORT_RDR:
                nearest_index_rdr = bisect.bisect(rdr_times, camera_time)

            if nearest_index_ldr >= 1 and (not EXPORT_RDR or nearest_index_rdr >=1):
                lidr_file = str(ldr_times[nearest_index_ldr - 1]) + '.ldr'
                if EXPORT_RDR:
                    radar_seq = str(rdr_times[nearest_index_rdr - 1]) + '.rdr'

                # Frames are indexed by 1, not
                real_frame = frame_number + 1

                if EXPORT_RDR:
                    self.frame_to_cloud_map[real_frame] = (lidr_file, radar_seq)
                else:
                    self.frame_to_cloud_map[real_frame] = lidr_file
                #print real_frame, (lidr_file, radar_seq)
        if write_to_file:
            self.__write_frame_map()


    def get_map(self):
        """ Returns a mapping from camera frame to ldr file """
        return self.frame_to_cloud_map

    def __write_frame_map(self):
        """ Writes the camera frame to ldr file mapping to a file """
        out_file = open(self.map_file_name, 'w')
        for frame, data in self.get_map().iteritems():
            line = str(frame) + ' ' + str(data[0])
            if EXPORT_RDR:
                line = str(frame) + ' ' + str(data[0]) + ' ' + str(data[1])
            else:
                line = str(frame) + ' ' + data
            line += '\n'
            out_file.write(line)
        out_file.close()

def utc_from_gps(gps_week, seconds, leap_seconds=16):
    """ Converts from gps week time to UTC time. UTC time starts from JAN 1,
        1970 and GPS time starts from JAN 6, 1980.

        http://leapsecond.com/java/gpsclock.htm
    """

    secs_in_week = 604800
    secs_gps_to_utc = 315964800

    return long((gps_week * secs_in_week + seconds + secs_gps_to_utc
                - leap_seconds) * 1000000)

def unpack_bag(basename, radar_bag_file):
    """ Unpacks the bag and writes individual segments to files. 
    The ouput folder is the basename + _rdr. 
    Each file name is the time of the starting segment 
    """
    radar_bag = rosbag.Bag(radar_bag_file)
    times = []
    cur_file = None
    rdr_dir = basename + '_rdr/'
    if not os.path.exists(rdr_dir):
        os.mkdir(rdr_dir)
    for topic, msg, t in radar_bag.read_messages(topics=['/object_list', '/target_status']):
        if msg.obj_id == 61:
            if cur_file != None:
                cur_file.close()
            time = msg.header.stamp.to_nsec()/1000 - 66000
            times.append(time)
            cur_file = open(rdr_dir + str(time) + '.rdr', 'w')
            
        if cur_file != None:
            if msg.obj_id == 0 or msg.obj_id == 62:
                continue
            line = None
            if topic == '/object_list':
                if msg.isMeasurd == True:
                    fmt = 'O {id} {dist} {lat_dist} {rel_spd} {dyn_prop} {rcs} {w} {l}'
                    line = fmt.format(
                        id = msg.obj_id, 
                        dist = msg.dist, 
                        lat_dist = msg.lat_dist,
                        rel_spd = msg.relative_spd,
                        dyn_prop = msg.dyn_prop,
                        rcs = msg.rcs,
                        w = msg.width,
                        l = msg.length) 
            else:
                if msg.status > 0:
                    fmt = 'T {id} {dist} {lat_dist} {rel_spd} {dyn_prop} {traj} {w} {l} {obst_probab} {exist_probab} {rel_acc} {type} {lost_reason}'
                    line = fmt.format(
                        id = msg.obj_id, 
                        dist = msg.dist, 
                        lat_dist = msg.lat_dist,
                        rel_spd = msg.relative_spd,
                        dyn_prop = msg.dyn_prop,
                        traj = msg.traj,
                        w = msg.width,
                        l = msg.length,
                        obst_probab = msg.obst_probab,
                        exist_probab = msg.exist_probab,
                        rel_acc = msg.relative_acc,
                        type = msg.type,
                        lost_reason = msg.lost_reason
                        )
            if line != None:
                cur_file.write(line + '\n')
    times.sort()
    return times

def main():
    """ Prints out times
    """

    if len(sys.argv) != 4:
        print """
        Usage: ./FrameFinder.py <gps_output_file> <ldr_folder_directory> <radar_bag_file>
        """
        sys.exit()

    gps_file = sys.argv[1]
    frames_folder = sys.argv[2]
    radar_folder = sys.argv[3]

    FrameFinder(gps_file, frames_folder, radar_folder, write_to_file=True)

if __name__ == '__main__':
    main()
