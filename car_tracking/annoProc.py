#!/usr/bin/python 

from AnnotationLib import *
from optparse import OptionParser

if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type="string", help='input annotation list (*.al or *.idl)', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type="string", help='directory for saving tracking results', default='./')
    parser.add_option('-f', '--first', dest='firstidx', type="int", help='first image to start tracking (0-based)', default=0)
    parser.add_option('-n', '--numimgs', dest='numimgs', type="int", help='number of images to process in the original image list', default=-1)

    (opts, args) = parser.parse_args()
    
    annolist_basedir = os.path.dirname(opts.annolist_name)
    annolist = parseXML(opts.annolist_name);

    # opts.firstidx = int(opts.firstidx);
    # opts.numimgs = int(opts.numimgs);

    if opts.numimgs == -1:
        numimgs = len(annolist);
    else:
        numimgs = opts.numimgs;

    firstidx = opts.firstidx;
    lastidx = opts.firstidx + numimgs - 1;

    print "processing images " + str(firstidx) + " to " + str(lastidx)

    annolist_out = annolist[firstidx:lastidx+1];

    annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
    annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);

    save_filename = opts.output_dir + "/" + annolist_base + "-subset";
    save_filename += "-firstidx" + str(firstidx) + "-lastidx" + str(lastidx)
    save_filename += annolist_ext;

    print "saving " + save_filename;
    saveXML(save_filename, annolist_out);

