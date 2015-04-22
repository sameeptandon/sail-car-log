import bitstring as bs
import rosbag
import sys, os
import multiprocessing
from collections import Counter
from mbly_obj_pb2 import Object, Objects

# Coordinate system: y grows up, x grows right, z is away

#~ 0x720-0x726 Sign recognition
#! 0x737 Lane information
#!! 0x738 Obstacle status (contains num_obstacles)
#!! 0x739+i*3 (i = num_obstacles -1) Obstacle data A
#!! 0x73A+i*3 Obstacle data B
#!! 0x73B+i*3 Obstacle data C
#! 0x669 Lane detail info
#! 0x766-0x767 Left Lane info
#! 0x768-0x769 Right Lane info
#! 0x76B number of lane markers
#! 0x76C + 2*N next lane A
#! 0x76D + 2*N next lane B

def run_command(args):
    print args
    (mbly_bag_file, out_folder) = args
    if out_folder[-1] != '/':
        out_folder += '/'
    out_name = out_folder + '/' + mbly_bag_file.replace('.bag', '.objproto')
    pb_objs = unpack_bag(out_folder, mbly_bag_file)
    with open(out_name, "wb") as f:
        f.write(pb_objs.SerializeToString())

def get_data(msg):
    return [bs.Bits(bytes=x) for x in msg.data]

def unpack_bag(out_folder, mbly_bag_file):
    """ Unpacks the bag and writes individual segments to files.
    The ouput folder is the basename + _mbly.
    *.obj contain objects; *.lane contain lane info
    Each file name is the time of the starting segment
    """
    basename = mbly_bag_file.split('.')[0] + '/'
    mbly_bag = rosbag.Bag(out_folder + mbly_bag_file)
    times = []
    cur_file = None
    mbly_dir = out_folder + basename

    objs = {}
    obj_cnt = -1
    obj_time = -1

    for topic, msg, t in mbly_bag.read_messages(topics=['/EyeData']):
        data = get_data(msg)

        if msg.id == 0x738:
            obj_cnt = data[0].uint
            obj_time = t.to_nsec()
            objs[obj_time] = []
        if obj_time != -1 and obj_cnt > 0:
            if msg.id >= 0x739 and msg.id <= 0x73B + (obj_cnt-1)*3:
                objs[obj_time].append(data)

    pb_objs = Objects()
    idx = 0
    for ts, data in objs.iteritems():
        # Objects have 3 data messages (A:0, B:1, C:2)
        # If there aren't 3 messages, skip this timestamp
        if len(data) % 3 != 0:
            continue
        for i in xrange(0, len(data), 3):
            # If we are slicing bitstrings, we need to reverse twice
            da = data[i]
            db = data[i + 1]
            dc = data[i + 2]

            pb_obj = pb_objs.object.add()
            pb_obj.timestamp = ts

            # Data from field A
            pb_obj.obj_id = da[0].int
            pb_obj.pos_x = bs.Bits().join(
                [da[2][::-1][:4][::-1], da[1]]).int * 0.0625
            pb_obj.pos_y = bs.Bits().join(
                [da[4][::-1][:2][::-1], da[3]]).int * 0.0625
            pb_obj.rel_vel_x = bs.Bits().join(
                [da[6][::-1][:4][::-1], da[5]]).int * 0.0625
            pb_obj.obj_type = da[6][::-1][4:7][::-1].uint
            pb_obj.status = da[7][::-1][:3][::-1].uint
            pb_obj.braking = int(da[7][::-1][3])
            pb_obj.location = da[4][::-1][5:7][::-1].uint
            pb_obj.blinker = da[4][::-1][2:5][::-1].uint
            pb_obj.valid = da[7][::-1][6:][::-1].uint

            # Data from field B
            pb_obj.length = db[0].uint * 0.05
            pb_obj.width = db[1].uint * 0.05
            pb_obj.age = db[2].uint
            pb_obj.lane = db[3][::-1][:2][::-1].uint

            # Data from field C
            pb_obj.acceleration_x = dc[4].int * 0.03
    return pb_objs

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print """Usage:
            generate_mbly.py <folder>"""
        exit(-1)

    target_dir = sys.argv[1]
    files = os.listdir(target_dir)
    radar_files = filter(lambda x: '_mbly.bag' in x,  files)

    pool = multiprocessing.Pool(processes=1)
    pool.map(run_command,
        zip(radar_files, [target_dir]*len(radar_files)))
    # run_command(zip(radar_files, [target_dir]*len(radar_files))[0])
