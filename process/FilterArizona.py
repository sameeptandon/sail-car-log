import cv2
import sys
import numpy as np
from VideoReader import VideoReader

mask = None

def constructMask(x, y, params):
    fs = params['frame_size']
    mask = np.zeros(fs, dtype=np.bool)
    mask[0:y, :, :] = 1
    for r in range(y, fs[0]):
        if x - (r - y) > 0:
            mask[r, 0:x-(r-y), :] = 1
        mask[r, x+(r-y):fs[1], :] = 1
    return mask


def mouseCallback(event, x, y, flags, params):
    global mask
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    mask = constructMask(x, y, params)


if __name__ == '__main__':
    video_file = sys.argv[1]
    mask_file = sys.argv[2]

    title = 'lane_anno'
    print video_file
    video_reader = VideoReader(video_file)

    k = 0
    while True:
        (success, I) = video_reader.getNextFrame()

        if not success:
            print 'End of video'
            break
        if k == 0:
            # Set up callback
            cv2.namedWindow(title, cv2.CV_WINDOW_AUTOSIZE)
            cv2.setMouseCallback(title, mouseCallback, {'frame_size': I.shape})

        if mask != None:
            I[mask] = 0
        cv2.imshow(title, I)
        cv2.waitKey(1)

        k += 1

    pass
