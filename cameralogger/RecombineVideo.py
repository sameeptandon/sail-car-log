import cv2, cv
import os

filename = "build/test1.avi"
num_splits = 2
framenum = 0

path, basename = os.path.split(filename)

names = [ ]
for j in range(num_splits):
    names.append(path + '/' + 'split_' + str(j) + '_' + basename)

captures = [ ]
for j in range(num_splits):
    captures.append(cv2.VideoCapture(names[j]))

recorder = cv2.VideoWriter("asdf.avi", cv.FOURCC('F','M','P','4'), 50.0, (1280, 960));
while True:
    framenum = framenum + 1
    ret, img = captures[framenum % num_splits].read()
    if ret == False:
        break
    recorder.write(img)

