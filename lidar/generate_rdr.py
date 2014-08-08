import os, sys
import multiprocessing
import rosbag

def run_command(args):
    print args
    (radar_bag_file, out_folder) = args
    if out_folder[-1] != '/':
        out_folder += '/'
    unpack_bag(out_folder, radar_bag_file)

def unpack_bag(out_folder, radar_bag_file):
    """ Unpacks the bag and writes individual segments to files.
    The ouput folder is the basename + _radar.
    Each file name is the time of the starting segment
    """
    basename = radar_bag_file.split('.')[0] + '/'
    radar_bag = rosbag.Bag(out_folder + radar_bag_file)
    times = []
    cur_file = None
    rdr_dir = out_folder + basename
    if not os.path.exists(rdr_dir):
        os.mkdir(rdr_dir)
    for topic, msg, t in radar_bag.read_messages(topics=['/object_list',
                                                         '/target_status']):
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
                    fmt = 'O {id} {dist} {lat_dist} {rel_spd} {dyn_prop} ' + \
                          '{rcs} {w} {l}'
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
                    fmt = 'T {id} {dist} {lat_dist} {rel_spd} {dyn_prop} ' + \
                          '{traj} {w} {l} {obst_probab} {exist_probab} ' + \
                          '{rel_acc} {type} {lost_reason}'
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

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print """Usage:
            generate_rdr.py <folder>"""
        exit(-1)

    target_dir = sys.argv[1]
    files = os.listdir(target_dir)
    radar_files = filter(lambda x: '_radar.bag' in x,  files)

    pool = multiprocessing.Pool(processes=1)
    pool.map(run_command,
        zip(radar_files, [target_dir]*len(radar_files)))
