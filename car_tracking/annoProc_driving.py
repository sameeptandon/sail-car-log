#!/usr/bin/python 

import pdb;
import copy
import math;
import numpy as np;

from AnnotationLib import *
from optparse import OptionParser

from helpers import *
import evaluate;

def filter_by_scale_annolist(annolist, min_width_px, max_width_px):
    num_rects = 0;

    for a in annolist:
        a.rects = filter_by_scale(a.rects, min_width_px, max_width_px);
        num_rects += len(a.rects);

    return num_rects;

def filter_by_scale(_rects, min_width_px, max_width_px):
    rects = copy.deepcopy(_rects);

    rects_filtered = [];
    num_rects = len(rects);

    for r in rects:
        cur_width = r.width();
        if cur_width >= min_width_px and cur_width < max_width_px:
            rects_filtered.append(r);
        
    return rects_filtered;


if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type='string', help='input annotation list (*.al or *.idl)', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type='string', help='directory for saving tracking results', default='./')
    parser.add_option('--sort', action="store_true", dest='do_sort_annolist', help='sort by frame index')

    parser.add_option('--eval_by_scale', action="store_true", dest='do_eval_scale', help='evluate for different scales')
    parser.add_option('-d', '--det_annolist', dest='det_annolist_name', type='string', help='detection annotation list (*.al or *.idl)', default=None)

    parser.add_option('--clip_path', action="store_true", dest='do_clip_path', help='clip path, make path relative to given depth')
    parser.add_option('--comp_stats', action="store_true", dest='do_comp_stats', help='compute statistics')

    parser.add_option('--clip_path_depth', dest='clip_path_depth', type="int", help='relative depth to clip', default=2)

    (opts, args) = parser.parse_args()
    
    annolist_basedir = os.path.dirname(opts.annolist_name)

    print "loading %s"  % opts.annolist_name;
    annolist = parse(opts.annolist_name);

    annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
    annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);

    do_save = False;

    if opts.do_sort_annolist:
        annolist_out = sorted(annolist, key = lambda a: get_image_prefix_idx(a.imageName));

        save_filename = opts.output_dir + "/" + annolist_base + "-sorted";
        save_filename += annolist_ext;
        
        do_save = True;

    elif opts.do_eval_scale:

        # MA: experiment, set score to bbox width 
        print "loading %s"  % opts.det_annolist_name;
        det_annolist = parseXML(opts.det_annolist_name);

        min_width = np.inf;
        max_width = -np.inf;

        for a in annolist:
            for r in a.rects:
                cur_width = r.width();

                if cur_width < min_width:
                    min_width = r.width();
                if cur_width > max_width:
                    max_width = r.width();


        print "min_width: %.2f, max_width: %.2f" % (min_width, max_width);

        if min_width < 20:
            min_width = 20;

        scale_range = np.linspace(min_width, max_width, 15);
        scale_num_rects = [];
        scale_recall = [];

        for idx in range(0, len(scale_range)-1):
            cur_annolist = copy.deepcopy(annolist);
            cur_det_annolist = copy.deepcopy(det_annolist);

            cur_num_rects = filter_by_scale_annolist(cur_annolist, scale_range[idx], scale_range[idx+1]);

            #saveXML("/afs/cs.stanford.edu/u/andriluka/code/cartrack_hossein/sail-car-log/car_tracking/tmp2/" + str(math.floor(scale_range[idx])) + "_" + str(math.floor(scale_range[idx+1]))a + ".al", cur_annolist);
            scale_num_rects.append(cur_num_rects);

            #filter_by_scale_annolist(cur_det_annolist, scale_range[idx], scale_range[idx+1]);
        
            #recalls, precs = evaluate.main(cur_annolist, cur_det_annolist);
            recalls, precs = evaluate.main(cur_annolist, det_annolist);
            scale_recall.append(recalls[-1]);
            #print "scale range: (%.2f, %.2f), recall: %.2f, precision: %.2f\n\n" % (scale_range[idx], scale_range[idx+1], recalls[-1], precs[-1]);

        
        recalls, precs = evaluate.main(annolist, det_annolist);
        print "recall: %.2f, precision: %.2f" % (recalls[-1], precs[-1]);

        print scale_range
        print scale_num_rects
        print scale_recall

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

        save_filename = opts.output_dir + "/" + annolist_base + "-relpath";
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
        save(save_filename, annolist_out);


