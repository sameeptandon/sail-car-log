#!/usr/bin/python 

import pdb;

from AnnotationLib import *
from optparse import OptionParser

from helpers import *

if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type='string', help='input annotation list (*.al or *.idl)', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type='string', help='directory for saving tracking results', default='./')
    parser.add_option('--sort', action="store_true", dest='do_sort_annolist', help='sort by frame index')
    parser.add_option('--clip_path', action="store_true", dest='do_clip_path', help='clip path, make path relative to given depth')
    parser.add_option('--comp_stats', action="store_true", dest='do_comp_stats', help='compute statistics')

    parser.add_option('--clip_path_depth', dest='clip_path_depth', type="int", help='relative depth to clip', default=2)


    (opts, args) = parser.parse_args()
    
    annolist_basedir = os.path.dirname(opts.annolist_name)
    annolist = parseXML(opts.annolist_name);

    annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
    annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);

    do_save = False;

    if opts.do_sort_annolist:
        annolist_out = sorted(annolist, key = lambda a: get_image_prefix_idx(a.imageName));

        save_filename = opts.output_dir + "/" + annolist_base + "-sorted";
        save_filename += annolist_ext;
        
        do_save = True;

    elif opts.do_clip_path:

        for a in annolist:
            n_found = 0;
            slashidx = -1;

            for idx in range(len(a.imageName)-1, -1, -1):
                if a.imageName[idx] == '/':
                    n_found += 1;
                
                if n_found == opts.clip_path_depth:
                    slashidx = idx;
                    break;

            if slashidx == -1:
                print "image %d, path is not deep enough, skipping ..." % idx
            else:
                a.imageName = a.imageName[slashidx+1:];

        annolist_out = annolist;

        save_filename = opts.output_dir + "/" + annolist_base + "-clip-path";
        save_filename += annolist_ext;

        do_save = True;

    elif opts.do_comp_stats:

        num_rects = 0;
        for a in annolist:
            num_rects += len(a.rects)

        print "number of images: %d "  % len(annolist);
        print "number or rectangles: %d " % num_rects

    else:
        print "unknown command, exiting ... ";
        exit();

    if do_save:
        print "saving " + save_filename;
        saveXML(save_filename, annolist_out);


