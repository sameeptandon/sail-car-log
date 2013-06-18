import cv2, cv
import os

filename = "build/collection2.avi"
num_splits = 10
framenum = 0

path, basename = os.path.split(filename)

names = [ ]
for j in range(num_splits):
    names.append(path + '/' + 'split_' + str(j) + '_' + basename)

captures = [ ]
for j in range(num_splits):
    captures.append(cv.CaptureFromFile(names[j]))
    print cv.GetCaptureProperty(captures[j], cv.CV_CAP_PROP_FRAME_COUNT)

recorder = cv.CreateVideoWriter("asdf.avi", cv.FOURCC('F','M','P','4'), 50.0, (1280, 960));
while True:
    framenum = framenum + 1
    img = cv.QueryFrame(captures[framenum % num_splits])
    if img is None:
        break
    print img
    cv.WriteFrame(recorder, img)
    print framenum

print framenum
