from pipeline_config import GPS_FILE
from GPSReader import GPSReader


def downsample_points(points, num_downsampled):
    step = len(points) / num_downsampled
    return points[0:len(points):step]


def read_lat_long(gps_file):
    gps_reader = GPSReader(gps_file)
    data = gps_reader.getNumericData()

    lat = data[:, gps_reader.token_order.index('lat')]
    lon = data[:, gps_reader.token_order.index('long')]
    return lat, lon


def construct_path_string(lat, lon, weight=5, color='0x0000ff'):
    return 'color:%s|weight:%d|' % (color, weight) +\
            '|'.join(['%f,%f' % pos for pos in zip(lat, lon)])


if __name__ == '__main__':
    api_key = open('api_key.txt', 'r').read().strip()

    lat, lon = read_lat_long(GPS_FILE)

    #lat_max, lat_min = max(lat), min(lat)
    #lon_max, lon_min = max(lon), min(lon)
    lat = downsample_points(lat, 25)
    lon = downsample_points(lon, 25)

    img_url = 'http://maps.googleapis.com/maps/api/staticmap?'
    params = {
        'key': api_key,
        'size': '500x500',
        'path': construct_path_string(lat, lon, weight=5, color='0x0000ff')
    }
    param_string = ''
    for key, val in params.iteritems():
        param_string += '&%s=%s' % (key, val)
    img_url += param_string[1:]
    print img_url
