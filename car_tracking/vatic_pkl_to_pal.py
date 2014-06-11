#!/usr/bin/python

import os;
import glob;

import AnnotationLib;

import pickle;
import pdb;

from optparse import OptionParser

if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-v', '--vatic_filename', dest='vatic_filename', type="string", help='input vatic dump in pkl format', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type="string", help='directory for saving output', default='./')
    parser.add_option('-i', '--images_dir', dest='images_dir', type="string", help='directory with images')
    #parser.add_option('-n', '--img_step', dest='imgstep', type="int", help='step between images', default=20)

    (opts, args) = parser.parse_args()

    print opts.vatic_filename;
    print opts.output_dir;
    print opts.images_dir;

    if not os.path.exists(opts.output_dir):
        print "ERROR: output dir not found";
        exit();
        
    if not os.path.exists(opts.images_dir):
        print "ERROR: images dir not found";
        exit();

    imglist = glob.glob(opts.images_dir + "/*.jpeg");
    imglist.sort();

    # create image list 
    annolist = [];
    for imgidx in xrange(0, len(imglist)):
        anno = AnnotationLib.Annotation()
        anno.imageName = imglist[imgidx];
        anno.rects = [];

        annolist.append(anno);
            
    # load vatic tracks 
    vatic_dump = pickle.load(open(opts.vatic_filename, "rb"));

    num_tracks = len(vatic_dump);
    print "number of tracks: ", num_tracks;

    for tidx in xrange(0, num_tracks):
        vatic_boxes = vatic_dump[tidx]["boxes"];
        track_len = len(vatic_boxes);
        print "track ", tidx, ", track_len: ", track_len;

        for bidx in xrange(0, track_len):
            if vatic_boxes[bidx].lost == 0:
                rect = AnnotationLib.AnnoRect()
                rect.id = tidx;
                rect.x1 = vatic_boxes[bidx].xtl;
                rect.y1 = vatic_boxes[bidx].ytl;
                rect.x2 = vatic_boxes[bidx].xbr;
                rect.y2 = vatic_boxes[bidx].ybr

                cur_frame = vatic_boxes[bidx].frame;
                annolist[cur_frame].rects.append(rect);


    # save annolist
    fname_ext = os.path.basename(opts.vatic_filename);
    fname = os.path.splitext(fname_ext)[0];

    pal_filename = opts.output_dir + "/" + fname + ".pal";
    print "saving ", pal_filename;
    AnnotationLib.save(pal_filename, annolist);


        

    # if not os.path.exists(opts.output_dir):
    #     print "creating ", opts.output_dir;
    #     os.makedirs(opts.output_dir);

    # imglist = glob.glob(opts.input_dir + "/*.jpeg");
    # imglist.sort();

    # idxlist = range(0, len(imglist), opts.imgstep);

    # print idxlist;

    # if idxlist[-1] != len(imglist) - 1:
    #     idxlist.append(len(imglist) - 1);

    # for idx in idxlist:
        
    #     #fname_ext = os.path.basename(imglist[idx]);
    #     #fname = os.path.splitext(fname_ext)[0];

    #     fname = str(idx).zfill(10);
    #     target_name = opts.output_dir + "/" + fname + ".jpg";

    #     print "copy ", imglist[idx], " to ", target_name

    #     shutil.copyfile(imglist[idx], target_name);
        


