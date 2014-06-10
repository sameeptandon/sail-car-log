#!/usr/bin/python 

import sys;

from AnnotationLib import *
from optparse import OptionParser

if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type="string", help='input annotation list (*.al or *.idl)', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type="string", help='directory for saving tracking results', default='./')
    parser.add_option('-f', '--first', dest='firstidx', type="int", help='first image to start tracking (0-based)', default=0)
    parser.add_option('-n', '--numimgs', dest='numimgs', type="int", help='number of images to process in the original image list', default=-1)
    parser.add_option('--step', dest='step', type="int", help='step between images used when generating subset of the original image list', default=-1)

    parser.add_option('--subset_with_last', action="store_true", dest='do_subset_with_last', help='subset of the image list')
    parser.add_option('--subset', action="store_true", dest='do_subset', help='subset of the image list')
    parser.add_option('--convert', dest='convert_name', type="string", help='convert/save to different format')

    parser.add_option('--min_width', dest='min_width', type="int", help='remove bounding boxes with width smaller than threshold', default=-1)
    parser.add_option('--max_width', dest='max_width', type="int", help='remove bounding boxes with width larger than threshold', default=-1)

    parser.add_option('--merge_sort', dest='merge_sort', type="string", help='merge two annotation lists and sort the result by filename')

    (opts, args) = parser.parse_args()

    annolist_basedir = os.path.dirname(opts.annolist_name)

    print "loading ", opts.annolist_name;
    annolist = parse(opts.annolist_name);

    # opts.firstidx = int(opts.firstidx);
    # opts.numimgs = int(opts.numimgs);

    if opts.do_subset or opts.do_subset_with_last:
        if opts.numimgs == -1:
            numimgs = len(annolist);
        else:
            numimgs = opts.numimgs;

        firstidx = opts.firstidx;
        lastidx = opts.firstidx + numimgs - 1;

        print "processing images " + str(firstidx) + " to " + str(lastidx)

        if opts.step != -1:
            annolist_out = annolist[firstidx:lastidx+1:opts.step];

            # always include the last element when generating a subset 
            if opts.do_subset_with_last:
                if annolist_out[-1].imageName != annolist[-1].imageName:
                    annolist_out.append(annolist[-1]);
            
        else:
            annoilst_out = annolist[firstidx:lastidx+1];

        annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
        annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);

        save_filename = opts.output_dir + "/" + annolist_base + "-subset";
        
        if firstidx != 0 or lastidx != len(annolist) - 1:
            save_filename += "-firstidx" + str(firstidx) + "-lastidx" + str(lastidx)

        if opts.step != -1: 
            if opts.do_subset_with_last:
                save_filename += "-steplast" + str(opts.step);
            else:
                save_filename += "-step" + str(opts.step);

        save_filename += annolist_ext;

        print "saving " + save_filename;
        save(save_filename, annolist_out);

    elif opts.merge_sort:

        merge_filename = opts.merge_sort;

        print "loading ", merge_filename;
        annolist_merge = parse(merge_filename);

        annolist = annolist + annolist_merge;

        annolist.sort(key=lambda a: (a.imageName));

        save_filename = opts.output_dir;

        fname_ext = os.path.splitext(save_filename)[1];
        print "format: ",fname_ext;

        if fname_ext == ".al" or fname_ext == ".pal":
            assert(save_filename != opts.annolist_name and save_filename != merge_filename)
            
            print "saving ", save_filename;
            save(save_filename, annolist);
        else:
            print "unrecognized output format";

    elif opts.min_width > 0 or opts.max_width > 0:

        min_width = opts.min_width;
        max_width = opts.max_width;

        if opts.max_width <= 0:
            max_width = 1e6;

        for a in annolist:
            a.rects = [r for r in a.rects if r.width() > min_width and r.width() < max_width];

        annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
        annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);

        save_filename = annolist_path + "/" + annolist_base;
        
        if opts.min_width != -1:
            save_filename += "-minwidth" + str(opts.min_width)

        if opts.max_width != -1:
            save_filename += "-maxwidth" + str(opts.max_width)

        save_filename += annolist_ext;

        print "saving " + save_filename;
        save(save_filename, annolist);

            

    elif opts.convert_name != None:
        print "saving ", opts.convert_name;
        save(opts.convert_name, annolist);

