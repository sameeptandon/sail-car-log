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

def inc_image_name(s, n):
    pos1 = s.rfind('_');
    pos2 = s.rfind('.');

    num = int(s[pos1+1:pos2]) + n
    numlen = pos2 - pos1 - 1;
    res = s[:pos1+1] + str(num).zfill(numlen) + s[pos2:];
    return res;

if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', help='annotation list (*.al or *.idl) used to inialize the tracking', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', help='directory for saving tracking results', default='./')
    parser.add_option('-f', '--first', dest='firstidx', help='first image to start tracking (0-based)', default=0)
    parser.add_option('-n', '--numimgs', dest='numimgs', help='number of images to process in the original image list', default=-1)
    
    (opts, args) = parser.parse_args()
    
    annolist_basedir = os.path.dirname(opts.annolist_name)
    annolist = parseXML(opts.annolist_name);

    opts.firstidx = int(opts.firstidx);
    opts.numimgs = int(opts.numimgs);

    if opts.numimgs == -1:
        numimgs = len(annolist);
    else:
        numimgs = opts.numimgs;

    firstidx = opts.firstidx;
    lastidx = opts.firstidx + numimgs - 1;

    print "tracking images " + str(firstidx) + " to " + str(lastidx)

    # convert to full path
    for a in annolist:
        a.imageName = annolist_basedir + "/" + a.imageName
        assert(os.path.isfile(a.imageName))

    annolist = annolist[firstidx:lastidx+1];

    annolist_track = [];
    
    trackMaxFrames = 50;

    for idx, a in enumerate(annolist):
        
        framesTracked = 0;
        num_missed_tracks = 0;
        num_skip_small = 0;

        curImageName = a.imageName;
	windows = [(rect.x1, rect.y1, rect.x2-rect.x1, rect.y2-rect.y1) for rect in a.rects];
        Img1 = cv2.imread(curImageName, 0);

        if not isinstance(Img1, np.ndarray):
            assert(False);

        print "frame: " + str(idx) + ", curImageName: " + curImageName

        while idx == len(annolist) - 1 or curImageName != annolist[idx+1].imageName:
            
            if not os.path.isfile(curImageName):
                print "image does not exist: " + curImageName
                assert(False);

            # add current image 
            a = Annotation();
            a.imageName = curImageName;

            for idx2 in range(len(windows)):
                r = AnnoRect();
                r.x1 = windows[idx2][0];
                r.y1 = windows[idx2][1];
                r.x2 = r.x1 + windows[idx2][2];
                r.y2 = r.y1 + windows[idx2][3];
                a.rects.append(r);

            annolist_track.append(a);

            # track to next image
            nextImageName = inc_image_name(curImageName, 1);
            print "\tnextImageName: " + nextImageName

            #
            # MA: it is possible that sequences ends at this point (e.g. if the input list contains multiple contactenated sequences)
            #
            if os.path.isfile(nextImageName):
                Img2 = cv2.imread(nextImageName, 0);
                assert(isinstance(Img2, np.ndarray))
                img_height = len(Img2)
                img_width = len(Img2[0]);

                assert(img_height > 0 and img_width > 0);
                next_windows = [];

                for idx2 in range(len(windows)): 
                    win = windows[idx2];

                    if win[2] <= 30 or win[3] <= 30:
                        num_skip_small += 1;
                        continue;
                    
                    nextWinInfo = NextRect(Img1, Img2, win);
                
                    if nextWinInfo[0]:
                        next_windows.append(nextWinInfo[1]);
                    else:
                        num_missed_tracks += 1;

                print "\tnum_active_tracks: " + str(len(next_windows)) + ", num_missed_tracks: " + str(num_missed_tracks) + ", num_skip_small: " + str(num_skip_small);

                framesTracked += 1;
            
                if framesTracked >= trackMaxFrames: 
                    break;

                curImageName = nextImageName;
                windows = next_windows;
                Img1 = Img2;

            else:
                print "End of sequence, could not find: " + nextImageName
                break;


                
    annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
    annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);
    
    save_filename = opts.output_dir + "/" + annolist_base + "-track";

    if opts.firstidx != 0 or opts.numimgs != -1:
        save_filename += "-firstidx" + str(firstidx) + "-lastidx" + str(lastidx)

    save_filename += annolist_ext;

    print "saving " + save_filename;
    saveXML(save_filename, annolist_track);
    
    
            
