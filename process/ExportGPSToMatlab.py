import os
import sys
from GPSReader import GPSReader
from scipy.io import savemat

def export(datafile):
    f = GPSReader(datafile)
    d = f.getNumericData()
    out = datafile.split('raw_data')
    outname = out[1][1:].replace('/','_')
    outname = outname.replace('.out', '.mat')
    savemat(outname, dict(data=d));

def main(rootdir): 
    fileList = []
    for root, subfolders, files in os.walk(rootdir):
        files = filter(lambda z: 'out' in z, files)
        for f in files:
            fileList.append(os.path.join(root,f))
    for f in fileList:
      export(f)

if __name__ == '__main__':
    main(sys.argv[1])
