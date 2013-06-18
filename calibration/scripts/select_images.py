"""
This script is for conveniently choosing which frames to save for calibration.
Before running this script, export the avi from the camera to images using the
following commands: 
    avconv -i <left camera avi> -f image2 left_%00d.bmp
    avconv -i <right camera avi> -f image2 right_%00d.bmp

Then run this script with the following usage:
python select_images.py <folder of images of left cam> <folder of images of
right cam> <output directory> 
"""

import sys
import os
import shutil
import cv

image_type = '.bmp'

def getAllImages(d):
    files = os.listdir(d); 
    return filter(lambda z: image_type in z, files)

if __name__ == '__main__':
    left_indir = sys.argv[1]
    right_indir = sys.argv[2]
    outdir = sys.argv[3]
    left_images = getAllImages(left_indir)
    left_images.sort(key = lambda z: int((z.split('_')[1]).split('.')[0]))
    right_images = getAllImages(right_indir)
    right_images.sort(key = lambda z: int((z.split('_')[1]).split('.')[0]))

    counter = 1
    for idx in range(len(left_images)):
        left_fname = left_indir + '/' + left_images[idx]
        right_fname = right_indir + '/' + right_images[idx]
        l_im = cv.LoadImage(left_fname, cv.CV_LOAD_IMAGE_COLOR);
        r_im = cv.LoadImage(right_fname, cv.CV_LOAD_IMAGE_COLOR);
        cv.ShowImage("left", l_im)
        cv.ShowImage("right", r_im)
        key = chr(cv.WaitKey())
        if key == 's':
            l_oname = outdir + '/' + 'left_' + str(counter) + image_type
            r_oname = outdir + '/' + 'right_' + str(counter) + image_type
            print l_oname
            print r_oname
            counter = counter + 1
            shutil.copy(left_fname, l_oname);
            shutil.copy(right_fname, r_oname);
