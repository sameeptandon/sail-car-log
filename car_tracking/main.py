#!/usr/bin/python 

import sys
import numpy as np
import cv2
import cv
import os.path

# debuging 
import pdb;

from trackingFunctions import *
from trackingFunctions import NextRect
from AnnotationLib import *
from optparse import OptionParser


if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type="string", help='annotation list (*.al or *.idl) used to inialize the tracking', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type="string", help='directory for saving tracking results', default='./')
    parser.add_option('-f', '--first', dest='firstidx', type="int", help='first image to start tracking (0-based)', default=0)
    parser.add_option('-n', '--numimgs', dest='numimgs', type="int", help='number of images to process in the original image list', default=-1)
    parser.add_option('--track_frames', dest='track_frames', type="int", help='number of frames to track', default=50)
    
    (opts, args) = parser.parse_args()
    
    annolist_basedir = os.path.dirname(opts.annolist_name)
    annolist = parseXML(opts.annolist_name);

    if opts.numimgs == -1:
        numimgs = len(annolist);
    else:
        numimgs = opts.numimgs;

    firstidx = opts.firstidx;
    lastidx = opts.firstidx + numimgs - 1;

    print "tracking images " + str(firstidx) + " to " + str(lastidx)

    # convert to full path
    for a in annolist:
        if not os.path.isabs(a.imageName):
            a.imageName = annolist_basedir + "/" + a.imageName

        assert(os.path.isfile(a.imageName))

    annolist = annolist[firstidx:lastidx+1];

    annolist_track = [];
    
    #trackMaxFrames = int(opts.track_frames);
    trackMaxFrames = opts.track_frames;

    for idx in range(len(annolist)):
        print "\n*** tracking starting at frame: " + str(idx)

        # track forward
        if idx < len(annolist) - 1:
            stop_imgname = annolist[idx+1].imageName;
        else:
            stop_imgname = inc_image_name(annolist[idx].imageName, trackMaxFrames);

        annolist_track_fwd = track_frame(annolist[idx], stop_imgname, trackMaxFrames, 1);

        # track backward
        if idx < len(annolist) - 1:
            stop_imgname = annolist[idx].imageName;
            annolist_track_back = track_frame(annolist[idx+1], stop_imgname, trackMaxFrames, -1);

            # merge two lists            
            #assert(len(annolist_track_fwd) == len(annolist_track_back));
            # MA: might happen that we can track forward but not backwards (e.g. when multiple sequences are concatenated in one file)
            if len(annolist_track_fwd) == len(annolist_track_back):

                annolist_track_back.reverse();

                for idx in range(1, len(annolist_track_fwd)):
                    # print annolist_track_fwd[idx].imageName
                    # print annolist_track_back[idx-1].imageName

                    assert(annolist_track_fwd[idx].imageName == annolist_track_back[idx-1].imageName);

                    #annolist_track_fwd[idx].rects += annolist_track_back[idx-1].rects;

                    r_new = [];

                    #MA: don't include duplicate rects 
                    for r_back in annolist_track_back[idx-1].rects:

                        found_similar = False;
                        for r_front in annolist_track_fwd[idx].rects:
                            min_iou = 0.65;
                            if r_back.overlap_pascal(r_front) > min_iou:
                                found_similar = True;
                                break;

                        if not found_similar:
                            r_new.append(r_back);

                    annolist_track_fwd[idx].rects += r_new;


        annolist_track += annolist_track_fwd;
            

    annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
    annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);
    
    save_filename = opts.output_dir + "/" + annolist_base + "-track";

    if opts.firstidx != 0 or opts.numimgs != -1:
        save_filename += "-firstidx" + str(firstidx) + "-lastidx" + str(lastidx) + "-numtrack" + str(trackMaxFrames)

    save_filename += annolist_ext;

    print "saving " + save_filename;
    saveXML(save_filename, annolist_track);
    
    
            
