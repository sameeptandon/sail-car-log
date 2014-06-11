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


def isTheSame(_rect1, _rect2):
   Thr = .5;
   rect1 = copy.deepcopy(_rect1);
   rect2 = copy.deepcopy(_rect2);

   intersection_rect = rect1.intersection(rect2)
   a1 = rect1.width() * rect1.height();
   a2 = rect2.width() * rect2.height();
   a0 = intersection_rect[0] * intersection_rect[1];
   if (a0 * 1.0) / a1 > Thr or (a0 * 1.0) / a2 > Thr:
	return True;
   else:
	return False

if __name__ == "__main__":

    min_remove_iou = 0.65;

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type="string", help='annotation list (*.al or *.idl) used to inialize the tracking', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type="string", help='directory for saving tracking results', default='./')
    parser.add_option('-f', '--first', dest='firstidx', type="int", help='first image to start tracking (0-based)', default=0)
    parser.add_option('-n', '--numimgs', dest='numimgs', type="int", help='number of images to process in the original image list', default=-1)
    parser.add_option('--track_frames', dest='track_frames', type="int", help='number of frames to track', default=50)
    
    (opts, args) = parser.parse_args()
    
    annolist_basedir = os.path.dirname(opts.annolist_name)
    annolist = parse(opts.annolist_name);

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
	    print a.imageName
            a.imageName = annolist_basedir + "/" + a.imageName
	    print "New->"+a.imageName;

    #assert(os.path.isfile(a.imageName))

    annolist = annolist[firstidx:lastidx+1];

    # MA: clip rectangles to image boundaries
    for a in annolist:
       for r in a.rects:
          if r.x1 < 0: 
             r.x1 = 0;
          if r.y1 < 0:
             r.y1 = 0;

    annolist_track = [];
    
    #trackMaxFrames = int(opts.track_frames);
    trackMaxFrames = opts.track_frames;

    # construct output filename
    annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
    annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);
    save_filename = opts.output_dir + "/" + annolist_base + "-track";

    if opts.firstidx != 0 or opts.numimgs != -1:
        save_filename += "-firstidx" + str(firstidx) + "-lastidx" + str(lastidx) + "-numtrack" + str(trackMaxFrames)

    save_filename_partiall = save_filename + "-partial" + annolist_ext;
    save_filename += annolist_ext;


    # do the tracking 

    for idx in range(len(annolist)):
        print "\n*** tracking starting at frame: " + str(idx)

        # track forward
        if idx < len(annolist) - 1:
            stop_imgname = annolist[idx+1].imageName;
        else:
            stop_imgname = inc_image_name(annolist[idx].imageName, trackMaxFrames);

        annolist_track_fwd = track_frame(annolist[idx], stop_imgname, trackMaxFrames, 1);

        annolist_track_main = [];

        # track backward
        if idx < len(annolist) - 1:
            stop_imgname = annolist[idx].imageName;
	    I = cv2.imread(stop_imgname);
	    
            annolist_track_back = track_frame(annolist[idx+1], stop_imgname, trackMaxFrames, -1);

            # merge two lists            
            #assert(len(annolist_track_fwd) == len(annolist_track_back));
            # MA: might happen that we can track forward but not backwards (e.g. when multiple sequences are concatenated in one file)
            if len(annolist_track_fwd) == len(annolist_track_back):
                annolist_track_back.reverse();

		annolist_len = len(annolist_track_fwd);
		midIdx = annolist_len/2;
		annolist_track_main = annolist_track_fwd[:midIdx+1] + annolist_track_back[midIdx:annolist_len-1];
		annolist_track_more = [annolist_track_fwd[0]] + annolist_track_back[0:midIdx] + annolist_track_fwd[midIdx+1:annolist_len]; 
		
                for idx2 in range(1, len(annolist_track_fwd)):
                    #print annolist_track_main[idx2].imageName
                    #print annolist_track_more[idx2].imageName

                    assert(annolist_track_main[idx2].imageName == annolist_track_more[idx2].imageName);

                    #annolist_track_fwd[idx2].rects += annolist_track_back[idx2-1].rects;

                    r_new = [];

                    #MA: don't include duplicate rects
			

                    for r_ind, r_more in enumerate(annolist_track_more[idx2].rects):

                        found_similar = False;
                        for r_main in annolist_track_main[idx2].rects:
                            if isTheSame(r_main, r_more):
				if r_ind > midIdx: r_main.classID = r_more.classID;
                                found_similar = True;
				if(r_ind < len(annolist_track_fwd)-2 and (r_main.x1 < 10 or r_main.x2 > I.shape[1]-10)):
					same_box_fouund = False;
					for next_frame_rect in annolist_track_main[r_ind+1].rects:
						if next_frame_rect.classID == r_main.classID:
							same_box_found = True;
							break;
					if same_box_found:
						if ((r_main.x1 < 10 and r_main.x1 < next_frame_rect.x1) or 
							(r_main.x2 > I.shape[1]-10 and r_main.x2 > next_frame_rect.x2)):
							r_main.x1 = r_more.x1;	
							r_main.x2 = r_more.x2;	
							r_main.y1 = r_more.y1;	
							r_main.y2 = r_more.y2;	
                                break;

                        if not found_similar:
                            r_new.append(r_more);

                    annolist_track_main[idx2].rects += r_new;

        # MA: add forward tracks if tracking back failed
        if len(annolist_track_main) == 0:
           annolist_track_main = annolist_track_fwd;

        annolist_track += annolist_track_main;
        
        # save results ones in a while 
        if idx % 10 == 0:
            print "saving " + save_filename_partiall;
            saveXML(save_filename_partiall, annolist_track);

    print "saving " + save_filename;
    saveXML(save_filename, annolist_track);
    
    
            
