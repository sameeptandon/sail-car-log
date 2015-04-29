import bitstring as bs
import rosbag
import sys, os
import multiprocessing
from collections import Counter
from mbly_obj_pb2 import Object, Objects
from mbly_lane_pb2 import Lane, Lanes
from mbly_ref_pb2 import Ref_pt, Ref_pts

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

can_id_to_lane_id_table = {
        0x766: -1, # ego left lane packet A
        0x767: -1, # ego left lane packet B
        0x768: 1, # ego right lane packet A
        0x769: 1, # ego right lane packet B
        0x76C: -2,
        0x76D: -2,
        0x76E: 2,
        0x76F: 2,
}

def run_command(args):
    print args
    (mbly_bag_file, out_folder) = args
    if out_folder[-1] != '/':
        out_folder += '/'

    out_obj_name = out_folder + '/' + mbly_bag_file.replace('.bag', '.objproto')
    out_lanes_name = out_folder + '/' + mbly_bag_file.replace('.bag', '.lanesproto')
    out_ref_pt_name = out_folder + '/' + mbly_bag_file.replace('.bag', '.refproto')

    (pb_objs, pb_lanes, pb_ref_pts) = unpack_bag(out_folder, mbly_bag_file)

    with open(out_obj_name, "wb") as f:
        f.write(pb_objs.SerializeToString())
    with open(out_lanes_name, "wb") as f:
        f.write(pb_lanes.SerializeToString())
    with open(out_ref_pt_name, "wb") as f:
        f.write(pb_ref_pts.SerializeToString())

def get_data(msg):
    return [bs.Bits(bytes=x) for x in msg.data]

def cut(data, start=0, end=8):
    """Gets the correct bits from the message. The mobileye manual lists bits in
    the reverse direction. To get bits as they are referred to in the manual,
    we need to flip, cut, then flip back.
    """
    return data[::-1][start:end][::-1]

def parse_objs(objs):
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
            pb_obj.obj_id = da[0].uint
            pb_obj.pos_x = (cut(da[2], 0, 4) + da[1]).uint * 0.0625
            pb_obj.pos_y = (cut(da[4], 0, 2) + da[3]).int * 0.0625
            pb_obj.rel_vel_x = (cut(da[6], 0, 4) + da[5]).int * 0.0625
            pb_obj.obj_type = cut(da[6], 4, 7).uint
            pb_obj.status = cut(da[7], 0, 3).uint
            pb_obj.braking = cut(da[7], 3, 4).uint
            pb_obj.location = cut(da[4], 5, 7).uint
            pb_obj.blinker = cut(da[4],2,5).uint
            pb_obj.valid = cut(da[7], 6, 8).uint

            # Data from field B
            pb_obj.length = db[0].uint * 0.05
            pb_obj.width = db[1].uint * 0.05
            pb_obj.age = db[2].uint
            pb_obj.lane = cut(db[3], 0, 2).uint

            # Data from field C
            pb_obj.acceleration_x = dc[4].int * 0.03
    return pb_objs

def parse_lanes(lanes):
    pb_lanes = Lanes()
    for ts, msgs in lanes.iteritems():
        msgs.sort(key = lambda x: x.id)

        # Lanes have 2 data messages
        # if there aren't 2, skip!
        if len(msgs) % 2 != 0:
            continue

        for i in xrange(0, len(msgs), 2):
            da = get_data(msgs[i])
            db = get_data(msgs[i+1])
            assert(msgs[i+1].id - msgs[i].id == 1)

            pb_lane = pb_lanes.lane.add()
            pb_lane.timestamp = ts
            pb_lane.lane_id = can_id_to_lane_id_table[msgs[i].id]
            pb_lane.C0 = (da[2] + da[1]).int  / 256.0
            pb_lane.C1 = ((db[1] + db[0]).uint - 0x7FFF)  / 1024.0
            pb_lane.C2 = ((da[4] + da[3]).uint - 0x7FFF) / (1024.0*1000)
            pb_lane.C3 = ((da[6] + da[5]).uint - 0x7FFF) / float(1 << 28)
            pb_lane.lane_type = cut(da[0], 0, 4).uint
            pb_lane.model_degree = cut(da[0], 6, 8).uint
            pb_lane.quality = cut(da[0], 4, 6).uint
            pb_lane.view_range_availability = cut(db[3], 7, 8).uint
            pb_lane.view_range = (cut(db[3],0,7) + db[2]).uint / 256.0
    return pb_lanes

def parse_ref_pts(ref_pts):
    pb_ref_pts = Ref_pts()
    for ts, msg in ref_pts.iteritems():
        da = get_data(msg)

        pb_ref_pt = pb_ref_pts.ref_pt.add()
        pb_ref_pt.timestamp = ts
        pb_ref_pt.position = ((da[1] + da[0]).uint - 0x7FFF) / 256.
        pb_ref_pt.distance = (cut(da[3], 0, 7) + da[2]).uint / 256.
        pb_ref_pt.is_valid = cut(da[3], 7, 8).uint
    return pb_ref_pts

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

    lanes = {}
    lane_time = -1

    ref_pts = {}
    for topic, msg, t in mbly_bag.read_messages(topics=['/EyeData']):
        data = get_data(msg)

        # obstacles
        if msg.id == 0x738:
            obj_cnt = data[0].uint
            obj_time = t.to_nsec()
            objs[obj_time] = []
        if obj_time != -1 and obj_cnt > 0:
            if msg.id >= 0x739 and msg.id <= 0x73B + (obj_cnt-1)*3:
                objs[obj_time].append(data)
        # lanes
        if msg.id == 0x766:
            lane_time = t.to_nsec()
            lanes[lane_time] = []
        if (msg.id >= 0x766 and msg.id <= 0x769) or \
            (msg.id >= 0x76c and msg.id <= 0x77f) and \
            lane_time >= 0:
                lanes[lane_time].append(msg)

        # reference points
        if msg.id == 0x76a:
            ref_time = t.to_nsec()
            ref_pts[ref_time] = msg


    pb_lanes = parse_lanes(lanes)
    pb_objs = parse_objs(objs)
    pb_ref_pts = parse_ref_pts(ref_pts)

    return (pb_objs, pb_lanes, pb_ref_pts)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print """Usage:
            generate_mbly.py <folder>"""
        exit(-1)

    target_dir = sys.argv[1]
    files = os.listdir(target_dir)
    radar_files = filter(lambda x: '_mbly.bag' in x,  files)

    pool = multiprocessing.Pool(processes=1)
    map(run_command,
        zip(radar_files, [target_dir]*len(radar_files)))
    # run_command(zip(radar_files, [target_dir]*len(radar_files))[0])
