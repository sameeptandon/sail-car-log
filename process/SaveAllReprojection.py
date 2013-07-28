import os
import sys
import subprocess
import multiprocessing


def command(arg):
  path, name = os.path.split(arg)
  outpath = path.replace('raw_data', 'raw_reproject')
  command = 'python SaveGPSReprojection.py %s %s 640 480' % (arg, outpath)

  return subprocess.call(command, shell=True)

def main(rootdir): 
    fileList = []
    for root, subfolders, files in os.walk(rootdir):
        files = filter(lambda z: 'split_0' in z, files)
        for f in files:
            fileList.append(os.path.join(root,f))

    arglist = [ ] 
    for f in fileList:
      path, fname = os.path.split(f)
      fname = fname[8:]
      arglist.append(path + '/' + fname)

    #command(arglist[0])
    pool = multiprocessing.Pool(processes=8)
    print pool.map(command, arglist)

if __name__ == '__main__':
    main(sys.argv[1])
