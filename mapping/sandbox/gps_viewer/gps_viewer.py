import os
import json
from pipeline_config import GPS_FILE, DATA_DIR
from GPSReader import GPSReader
from collections import defaultdict


def downsample_points(points, num_downsampled):
    if len(points) < num_downsampled:
        return points
    step = len(points) / num_downsampled
    return points[0:len(points):step]


def read_lat_lon(gps_file):
    gps_reader = GPSReader(gps_file)
    data = gps_reader.getNumericData()

    lat = data[:, gps_reader.token_order.index('lat')]
    lon = data[:, gps_reader.token_order.index('long')]
    return lat, lon


if __name__ == '__main__':
    API_KEY = open('api_key.txt', 'r').read().strip()

    SCAIL_Q50_DATA_DIR = '/scail/group/deeplearning/driving_data/q50_data'
    route_segment_split_gps_files = defaultdict(lambda: defaultdict(dict))
    routes = [d for d in os.listdir(SCAIL_Q50_DATA_DIR) if os.path.isdir(d)]
    for route in routes:
        route_dir = os.path.join(SCAIL_Q50_DATA_DIR, route)
        gps_out_files = [f for f in os.listdir(route_dir) if f.endswith('gps.out')]
        segments = list()
        for gps_out_file in gps_out_files:
            parts = gps_out_file.split('_')
            split = [part for part in parts if (len(part) == 1 and part.isalpha())][0]
            segment = parts[0:parts.index(split)]
            route_segment_split_gps_files[route][segment][split] =\
                {'gps_file': os.path.join(route_dir, gps_out_file)}

    gps_files = list()
    gps_files.append(GPS_FILE)
    gps_files.append('%s/280N_a2/280N_a_gps.out' % DATA_DIR)
    gps_files.append('%s/sandhill_b2/sandhill_b_gps.out' % DATA_DIR)

    track_names = ['17N_b', '280N_a', 'sandhill_b']

    gps_tracks = dict()
    lat_max_all, lat_min_all = float('-inf'), float('inf')
    lon_max_all, lon_min_all = float('-inf'), float('inf')

    for k in range(len(gps_files)):
        gps_file = gps_files[k]
        track_name = track_names[k]

        lat, lon = read_lat_lon(gps_file)
        lat = downsample_points(lat, 1000)
        lon = downsample_points(lon, 1000)

        gps_tracks[track_name] = {'lat': lat.tolist(), 'lon': lon.tolist()}

        lat_max, lat_min = max(lat), min(lat)
        lon_max, lon_min = max(lon), min(lon)
        lat_max_all, lat_min_all = max(lat_max, lat_max_all), min(lat_min, lat_min_all)
        lon_max_all, lon_min_all = max(lon_max, lon_max_all), min(lon_min, lon_min_all)

    lat_center = (lat_max_all + lat_min_all) / 2
    lon_center = (lon_max_all + lon_min_all) / 2

    template = open('viewer_template.html', 'r').read()
    output = template.format(api_key=API_KEY, lat_center=lat_center, lon_center=lon_center)
    open('viewer.html', 'w').write(output)

    json.dump(gps_tracks, open('gps_tracks.json', 'w'))
