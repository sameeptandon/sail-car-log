import os
import json
from gps_viewer import get_route_segment_split_gps
from match_traces import read_gps_ecef, trace_dir, get_bbox, deg_between,\
        bbox_overlap
from collections import defaultdict

'''
Go through unlabeled runs and assign each (route, segment, split)  to a
freeway using previous labels
'''

if __name__ == '__main__':
    labeled_data = json.load(open('segment_freeways.json', 'r'))

    unlabeled_rss = list()
    labeled_rss = list()
    labeled_bboxes = list()
    labeled_dirs = list()
    labeled_freeways = list()

    route_segment_split_gps = get_route_segment_split_gps()

    # First figure out what's labeled and what's not
    for route in route_segment_split_gps:
        segment_split_gps = route_segment_split_gps[route]
        for segment in segment_split_gps:
            split_gps = segment_split_gps[segment]
            for split in split_gps:
                if route not in labeled_data or segment not in labeled_data[route]\
                        or split not in labeled_data[route][segment]:
                    # Need to label the gps file
                    print route, segment, split
                    unlabeled_rss.append((route, segment, split))
                else:
                    xyz = read_gps_ecef(split_gps[split]['gps_file'])
                    print split_gps[split]['gps_file']
                    if xyz is None:
                        continue
                    labeled_rss.append((route, segment, split))
                    labeled_bboxes.append(get_bbox(xyz))
                    labeled_dirs.append(trace_dir(xyz))
                    labeled_freeways.append(labeled_data[route][segment][split])

    assert len(labeled_dirs) == len(labeled_bboxes) == len(labeled_rss)

    route_segment_split_freeway = defaultdict(lambda: defaultdict(dict))
    # Then figure out which of the labeled traces in same direction
    # overlaps the most with unlabeled trace and assign it that freeway
    for urss in unlabeled_rss:
        route, segment, split = urss
        gps_file = route_segment_split_gps[route][segment][split]['gps_file']
        if not os.path.exists(gps_file):
            print 'No GPS file:', gps_file
            continue
        uxyz = read_gps_ecef(gps_file)
        udir = trace_dir(uxyz)
        ubbox = get_bbox(uxyz)

        overlaps = list()
        for k in range(len(labeled_rss)):
            # PARAM
            if deg_between(udir, labeled_dirs[k]) > 90:
                overlaps.append(-1)
                continue
            overlaps.append(bbox_overlap(ubbox, labeled_bboxes[k], padding=0)[2])

        print overlaps

        if max(overlaps) > 0.25:  # PARAM
            freeway = labeled_freeways[overlaps.index(max(overlaps))]
        else:
            freeway = 'none'
        print urss, freeway, max(overlaps)

        route_segment_split_freeway[route][segment][split] = freeway

    # Finally print out JSON dictionary to add to labeled data

    print json.dumps(route_segment_split_freeway, sort_keys=True, indent=4)
