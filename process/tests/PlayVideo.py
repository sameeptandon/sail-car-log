from VideoReader import *
from ArgParser import *
import sys, cv2

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])

    write = True

    fourcc = cv2.cv.CV_FOURCC(*'MPG1')
    vid_name = '/deep/u/jkiske/test.mpg'
    writer = None
    fnum = 0

    with VideoReader(args['video']) as  reader:
        while True:
            (success, I) = reader.getNextFrame()
            if not success:
                break
            if write == True:
                if writer == None:
                    writer = cv2.VideoWriter(vid_name, fourcc, 50,
                                             (I.shape[1], I.shape[0]))
                writer.write(I)
                fnum += 1
                print fnum
                if fnum >= 50:
                    break
            else:
                cv2.imshow('video', I)
                cv2.waitKey(1)
