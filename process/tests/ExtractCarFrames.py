from VideoReader import * 
from ArgParser import *
import sys, cv2, pickle, os, stat

FRAME_STEP = 5
HISTORY_WINDOW = 40 
FUTURE_WINDOW = 30

def export_frames(fnums, fname):
    f = open(fname, 'w')
    pickle.dump(fnums, f)


def drawBorder(I, color, thickness): 
    cv2.rectangle(I, (0,0), (I.shape[1], I.shape[0]), color, thickness)

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    reader = VideoReader(args['video'])

    outdir = sys.argv[1] + '/car_frames'
    try:
        os.mkdir(outdir)
    except:
        pass
    os.chmod(outdir, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IWGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IWOTH | stat.S_IXOTH )

    outfname = outdir + "/" + args['basename'] + str(args['cam_num']) + '.pickle'


    framenums = set()
    last_record = 0 
    while True:
        for p in range(FRAME_STEP):
            (success, I) = reader.getNextFrame()
        if not success:
            break

        if reader.framenum in framenums:
            drawBorder(I, (0,255,0), 10)
        cv2.imshow('video', cv2.pyrDown(cv2.pyrDown(I)))
        key = chr(cv2.waitKey(5) & 255)
        if key == 'c':
            print reader.framenum
            for p in range(HISTORY_WINDOW):
                framenums.add(reader.framenum - p)
            for p in range(FUTURE_WINDOW):
                framenums.add(reader.framenum + p)

    export_frames(framenums, outfname)
