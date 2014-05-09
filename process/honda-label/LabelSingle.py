import os
import sys
import subprocess
import multiprocessing


def command(args):
  (arg, output_folder) = args
  path, name = os.path.split(arg)
  output_prefix = path.replace('/','_')
  name, ext = os.path.splitext(name)
  all_points = os.path.join(output_folder, '%s-%s-all-points.mat' % (output_prefix, name))
  scores = os.path.join(output_folder, '%s-%s-interpolated.mat' % (output_prefix, name))

  command = 'python LaneClickTest.py %s %s --quiet; ' % (arg, all_points)
  command += 'python interp_2d.py %s %s' % (all_points, scores)

  return subprocess.call(command, shell=True)

def main(rootdir, output_folder): 
    fileList = []
    for root, subfolders, files in os.walk(rootdir):
        files = filter(lambda z: 'split_0' in z, files)
        for f in files:
            fileList.append(os.path.join(root,f))

    prefix = len(os.path.commonprefix(fileList))
    arglist = [ ] 
    for f in fileList:
      path, fname = os.path.split(f)
      fname = fname[8:]
      arglist.append((path + '/' + fname, output_folder))

    #command(arglist[0])
    pool = multiprocessing.Pool(processes=12)
    print pool.map(command, arglist)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
