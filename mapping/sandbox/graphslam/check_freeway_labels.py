import json
from gps_viewer import get_route_segment_split_gps
from collections import defaultdict

if __name__ == '__main__':
    all_freeways = defaultdict(int)
    data = json.load(open('segment_freeways.json', 'r'))

    route_segment_split_gps = get_route_segment_split_gps()
    for route in route_segment_split_gps:
        segment_split_gps = route_segment_split_gps[route]
        for segment in segment_split_gps:
            split_gps = segment_split_gps[segment]
            for split in split_gps:
                print route, segment, split
                freeways= data[route][segment][split].split('+')
                for freeway in freeways:
                    if freeway == '':
                        continue
                    all_freeways[freeway] += 1

    print all_freeways.keys()
    print len(all_freeways.keys())
    print all_freeways
