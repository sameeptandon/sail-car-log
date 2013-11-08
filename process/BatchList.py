import os
import sys
import subprocess
import multiprocessing

def main(rootdir, out_folder, pickle_loc, exclude_list, test): 
    root_to_prefix = rootdir if rootdir[-1] is not '/' else rootdir[:-1]
    pref = os.path.basename(root_to_prefix) + '_'
    visited_prefix = set([])
    fileList = []
    for root, subfolders, files in os.walk(rootdir):
        files = filter(lambda z: 'split_0' in z, files)
        for f in files:
            fileList.append(os.path.join(root,f))

    prefix = len(os.path.commonprefix(fileList))
    arglist = [ ] 
    for f in fileList:
      path, fname = os.path.split(f)
      fname = fname[8:-5]
      prefix = path + '/' + fname
      if (prefix + '1.avi') in exclude_list:
          print 'skipping: %s' % prefix
          continue
      pts_base = path.replace('/', '_') + '-'
      if prefix not in visited_prefix:
        arglist.append((prefix, out_folder, pref, os.path.join(pts_loc, pts_base), pickle_loc))
        visited_prefix.add(prefix)

    # run parseFolder analog over each set of arguments
    #command(arglist[0])
    #pool = multiprocessing.Pool(processes=12)
    #print pool.map(parseFolder, arglist)

if __name__ == '__main__':
    f = open(sys.argv[4], 'rb')
    lines = f.readlines()
    f.close()
    test = False
    for i in xrange(len(lines)):
        lines[i] = lines[i].strip()
    main(sys.argv[1], sys.argv[2], sys.argv[3], lines, test)
