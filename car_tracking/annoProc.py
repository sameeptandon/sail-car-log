#!/usr/bin/python 

from AnnotationLib import *
from optparse import OptionParser

if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type="string", help='input annotation list (*.al or *.idl)', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type="string", help='directory for saving tracking results', default='./')
    parser.add_option('-f', '--first', dest='firstidx', type="int", help='first image to start tracking (0-based)', default=0)
    parser.add_option('-n', '--numimgs', dest='numimgs', type="int", help='number of images to process in the original image list', default=-1)

    parser.add_option('--subset', action="store_true", dest='do_subset', help='subset of the image list')
    parser.add_option('--convert', dest='convert_name', type="string", help='convert/save to different format')

    (opts, args) = parser.parse_args()
   
    annolist_basedir = os.path.dirname(opts.annolist_name)

    print "loading ", opts.annolist_name;
    annolist = parse(opts.annolist_name);

    # opts.firstidx = int(opts.firstidx);
    # opts.numimgs = int(opts.numimgs);


    if opts.do_subset:
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
        save(save_filename, annolist_out);


    elif opts.convert_name != None:
        print "saving ", opts.convert_name;
        save(opts.convert_name, annolist);

