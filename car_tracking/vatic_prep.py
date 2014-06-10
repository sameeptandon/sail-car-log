#!/usr/bin/python

import os;
import glob;

import shutil;

from optparse import OptionParser


if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-i', '--input_dir', dest='input_dir', type="string", help='input directory', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type="string", help='directory for saving tracking results', default='./')
    parser.add_option('-n', '--img_step', dest='imgstep', type="int", help='step between images', default=20)

    (opts, args) = parser.parse_args()

    print opts.input_dir;
    print opts.output_dir;

    if not os.path.exists(opts.output_dir):
        print "creating ", opts.output_dir;
        os.makedirs(opts.output_dir);

    imglist = glob.glob(opts.input_dir + "/*.jpeg");
    imglist.sort();

    idxlist = range(0, len(imglist), opts.imgstep);

    print idxlist;

    if idxlist[-1] != len(imglist) - 1:
        idxlist.append(len(imglist) - 1);

    for idx in idxlist:
        
        #fname_ext = os.path.basename(imglist[idx]);
        #fname = os.path.splitext(fname_ext)[0];

        fname = str(idx).zfill(10);
        target_name = opts.output_dir + "/" + fname + ".jpg";

        print "copy ", imglist[idx], " to ", target_name

        shutil.copyfile(imglist[idx], target_name);
        


