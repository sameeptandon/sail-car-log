import os
import json
from GPSReader import GPSReader
from collections import defaultdict
from pipeline_config import SCAIL_Q50_DATA_DIR


def downsample_points(points, num_downsampled):
    if len(points) < num_downsampled:
        return points
    step = len(points) / num_downsampled
    return points[0:len(points):step]


def read_gps_fields(gps_file, fields):
    gps_reader = GPSReader(gps_file)
    data = gps_reader.getNumericData()

    field_data = list()
    for field in fields:
        field_data.append(data[:, gps_reader.token_order.index(field)])
    return field_data


def get_route_segment_split_gps():
    route_segment_split_gps = defaultdict(lambda: defaultdict(dict))
    routes = [d for d in os.listdir(SCAIL_Q50_DATA_DIR)]
    for route in routes:
        route_dir = os.path.join(SCAIL_Q50_DATA_DIR, route)
        gps_out_files = [f for f in os.listdir(route_dir) if f.endswith('gps.out')]
        for gps_out_file in gps_out_files:
            parts = gps_out_file.split('_')
            split = [part for part in parts if (len(part) == 1 and part.isalpha())][0]
            segment = '_'.join(parts[0:parts.index(split)])
            route_segment_split_gps[route][segment][split] =\
                {'gps_file': os.path.join(route_dir, gps_out_file)}
    return route_segment_split_gps


if __name__ == '__main__':
    API_KEY = open('api_key.txt', 'r').read().strip()

    route_segment_split_gps = get_route_segment_split_gps()

    lat_max_all, lat_min_all = float('-inf'), float('inf')
    lon_max_all, lon_min_all = float('-inf'), float('inf')

    for route in route_segment_split_gps:
        print route
        segment_split_gps = route_segment_split_gps[route]
        for segment in segment_split_gps:
            print '\t' + segment
            split_gps = segment_split_gps[segment]
            for split in split_gps:
                gps_file = split_gps[split]['gps_file']
                print '\t\t' + split + ': ' + gps_file

                lat, lon, _ = read_gps_fields(gps_file, ['lat', 'long', 'height'])
                lat = downsample_points(lat, 1000)
                lon = downsample_points(lon, 1000)
                if len(lat) == 0:
                    continue

                split_gps[split]['lat'] = lat.tolist()
                split_gps[split]['lon'] = lon.tolist()

                lat_max, lat_min = max(lat), min(lat)
                lon_max, lon_min = max(lon), min(lon)
                lat_max_all, lat_min_all = max(lat_max, lat_max_all), min(lat_min, lat_min_all)
                lon_max_all, lon_min_all = max(lon_max, lon_max_all), min(lon_min, lon_min_all)

    lat_center = (lat_max_all + lat_min_all) / 2
    lon_center = (lon_max_all + lon_min_all) / 2

    template = open('viewer_template.html', 'r').read()
    output = template.format(api_key=API_KEY, lat_center=lat_center, lon_center=lon_center)
    open('viewer.html', 'w').write(output)

    json.dump(route_segment_split_gps, open('gps_tracks.json', 'w'))
