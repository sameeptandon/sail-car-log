#!/usr/bin/python 

import cv;
import cv2;
import os;
import sys;

from AnnotationLib import *
from optparse import OptionParser

from helpers import *

if __name__ == "__main__":

    parser = OptionParser();

    parser.add_option('-a', '--annolist', dest='annolist_name', type='string', help='input annotation list (*.al or *.idl)', default=None)
    parser.add_option('-o', '--output_dir', dest='output_dir', type='string', help='directory for saving tracking results', default='./')
    parser.add_option('-f', '--first', dest='firstidx', type="int", help='first image to start tracking (0-based)', default=0)
    parser.add_option('-n', '--numimgs', dest='numimgs', type="int", help='number of images to process in the original image list', default=-1)

    (opts, args) = parser.parse_args()

    annolist_basedir = os.path.dirname(opts.annolist_name)
    #annolist = parseXML(opts.annolist_name);
    annolist = parse(opts.annolist_name);

    if opts.numimgs == -1:
        numimgs = len(annolist);
    else:
        numimgs = opts.numimgs;

    firstidx = opts.firstidx;
    lastidx = opts.firstidx + numimgs - 1;

    if lastidx > len(annolist) - 1:
        lastidx = len(annolist) - 1;

    annolist = annolist[firstidx:lastidx+1];

    print "processing images " + str(firstidx) + " to " + str(lastidx)
    
    annolist_path, annolist_base_ext = os.path.split(opts.annolist_name);
    annolist_base, annolist_ext = os.path.splitext(annolist_base_ext);

    for a in annolist:
        if not os.path.isabs(a.imageName):
            a.imageName = annolist_basedir + "/" + a.imageName

        assert(os.path.isfile(a.imageName))

    save_filename = opts.output_dir + "/" + annolist_base;

    if opts.firstidx != 0 or opts.numimgs != -1:
        save_filename += "-firstidx" + str(firstidx) + "-lastidx" + str(lastidx);

    save_filename += ".avi";

    assert(len(annolist) > 0);

    Img1 = cv2.imread(annolist[0].imageName);
    height, width, layers =  Img1.shape;
    
    print (height, width, layers)

    fourcc = cv2.cv.CV_FOURCC('F','M','P', '4')

    print 'total images: %d' % len(annolist);
    print 'video filename: %s' % save_filename;

    out = cv2.VideoWriter(save_filename, fourcc, 20.0, (width, height))
    assert out.isOpened()

    for idx, a in enumerate(annolist):
        I = cv2.imread(a.imageName);
        
	if a.rects != []:
            for r in a.rects:
		cv2.rectangle(I, (int(r.x1),int(r.y1)), (int(r.x2), int(r.y2)), 255, 2)

        out.write(I);
        
        if (idx + 1) % 10 == 0:
            sys.stdout.write('.');
            sys.stdout.flush();

        if (idx + 1) % 800 == 0:
            sys.stdout.write('\n');
            sys.stdout.flush();
    

    sys.stdout.write('\n');

   # Release everything if job is finished
    out.release()
    cv2.destroyAllWindows()
 
