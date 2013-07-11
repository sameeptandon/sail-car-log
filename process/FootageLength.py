import os
import sys
from GPSReader import GPSReader


def numseconds(datafile):
    f = GPSReader(datafile)
    d = f.getData()
    first =  d[0]['seconds']
    last = d[-1]['seconds']
    return last - first


def main(rootdir): 
    fileList = []
    for root, subfolders, files in os.walk(rootdir):
        files = filter(lambda z: 'out' in z, files)
        for f in files:
            fileList.append(os.path.join(root,f))
    total = 0
    for f in fileList:
        seconds = numseconds(f)
        total += seconds
        path, name = os.path.split(f)
        print name, seconds / 60, 'minutes'

    print 'total =', total/60, 'minutes'

if __name__ == '__main__':
    main(sys.argv[1])
