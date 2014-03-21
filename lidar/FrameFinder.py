#!/usr/bin/python
# -*- coding: utf-8 -*-

import bisect
import os
import sys

sys.path.append('../process')
from GPSReader import GPSReader


class FrameFinder:

    """ Creates a mapping from cloud to frames. If the file
    "gps_file_name_to_frame_folder_name.map" exists, it will read from that
    file. It is about 2x faster to read from this file"
    """

    frame_to_cloud_map = {}
    cloud_to_frame_map = {}
    map_file_path = ""
    map_file_name = ""

    def __init__(self, gps_file_path, frames_folder, write_to_file):
        gps_file_name = os.path.basename(gps_file_path).split('.')[0]
        if frames_folder[-1] == '/':
            frames_folder = frames_folder[0:-1]
        basename = frames_folder.replace('_frames', '')
        frames_folder_name = os.path.basename(frames_folder)
        self.map_file_name = basename + ".map"

        if os.path.isfile(self.map_file_name):
           self.__read_frame_map()
        else:
            reader = GPSReader(gps_file_path)

            # Camera time should already be sorted because frame # always increases
            camera_times = [utc_from_gps(data['week'], data['seconds'])
                            for data in reader.getData()]

            ldr_times = \
                [long(os.path.splitext(os.path.basename(ldr_file))[0])
                 for ldr_file in os.listdir(frames_folder)
                 if ldr_file.endswith('.ldr')]

            # Need to sort the ldr_times because listdir returns undefined ordering
            ldr_times.sort()

            for frame_number, camera_time in enumerate(camera_times):
                # Find the closest time in ldr times
                nearest_index = bisect.bisect(ldr_times, camera_time)
                if nearest_index >= 1:
                    lidr_file = str(ldr_times[nearest_index - 1]) + '.ldr'
                    # Frames are indexed by 1, not
                    real_frame = frame_number + 1
                    self.frame_to_cloud_map[real_frame] = lidr_file
                    self.cloud_to_frame_map[lidr_file] = real_frame
            
            if write_to_file == True:
                self.__write_frame_map()


    def get_map(self):
        """ Returns a mapping from camera frame to ldr file """
        return self.frame_to_cloud_map

    def get_inverse_map(self):
        """ Returns a mapping from cloud to frame """
        return self.cloud_to_frame_map

    def __write_frame_map(self):
        """ Writes the camera frame to ldr file mapping to a file """
        out_file = open(self.map_file_name, 'w')
        for frame, cloud in self.get_map().iteritems():
            out_file.write(str(frame) + " " + str(cloud) + "\n")
        out_file.close()

    def __read_frame_map(self):
         for line in open(self.map_file_name, 'r'):
                tokens = line.split(" ")
                frame = tokens[0]
                cloud = tokens[1]
                self.frame_to_cloud_map[frame] = cloud
                self.cloud_to_frame_map[cloud] = frame

def utc_from_gps(gps_week, seconds, leap_seconds=16):
    """ Converts from gps week time to UTC time. UTC time starts from JAN 1,
        1970 and GPS time starts from JAN 6, 1980.

        NOTE: I am not sure if posix corrects for leap seconds, I am leaving
        it in for now just in case. If the times are 16 seconds off look here

        http://leapsecond.com/java/gpsclock.htm
    """

    secs_in_week = 604800
    secs_gps_to_utc = 315964800

    return long((gps_week * secs_in_week + seconds + secs_gps_to_utc
                - leap_seconds) * 1000000)


def main():
    """ Prints out times
    """

    if len(sys.argv) != 3:
        print """
        Usage: ./match_frames.py <gps_output_file> <ldr_folder_directory>
        """
        sys.exit()

    gps_file_path = sys.argv[1]
    frames_folder = sys.argv[2]

    finder = FrameFinder(gps_file_path, frames_folder, True)

if __name__ == '__main__':
    main()
