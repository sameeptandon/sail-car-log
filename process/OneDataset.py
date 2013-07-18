from LabelGenerator import *
import sys

if __name__ == '__main__':
    file_name = sys.argv[1]
    gps_name = sys.argv[2]
    output = sys.argv[3]
    start_num = int(sys.argv[4]) if len(sys.argv) > 4 else 0
    end_num = int(sys.argv[5]) if len(sys.argv) > 4 else -1

    runLabeling(file_name, gps_name, output, start_num, end_num)
