import bisect
import cv2
import glob
import numpy as np
import sys

from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms, absoluteTransforms
from LaneMarkingHelper import get_transforms, mk2_to_mk1, BackProjector
from LidarTransforms import R_to_c_from_l, utc_from_gps_log_all
from VideoReader import VideoReader

class ImageClicker(object):

    def __init__(self):
        args = parse_args(sys.argv[1], sys.argv[2])

        planar = glob.glob(sys.argv[1] + '/*_planar.npz')[0]
        npz = np.load(planar)
        self.planes = npz['planes']

        (self.imu_transforms_mk1,
         self.gps_data_mk1,
         self.gps_times_mk1) = get_transforms(args, 'mark1')

        (self.imu_transforms_mk2,
         self.gps_data_mk2,
         self.gps_times_mk2) = get_transforms(args, 'mark2')

        self.back_projector = BackProjector(args)
        self.vr = VideoReader(args['video'])

        self.t = 1
        cv2.namedWindow('image', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('image', self.mouseHandler)

    def mouseHandler(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            pix = np.array([x, y, 1])
            mk1_t = mk2_to_mk1(self.t, self.gps_times_mk1, self.gps_times_mk2)

            xform = self.imu_transforms_mk1[mk1_t]

            v = self.back_projector.calculateBackProjection(xform, pix)
            l0 = self.back_projector.calculateBackProjection(xform)

            l = l0 - v

            best_model = (np.inf, 0, 0)
            for i, plane in enumerate(self.planes):
                pt = self.back_projector.calculateIntersection(l0, l,
                                                               plane)
                d = np.linalg.norm(pt - plane[3:])
                if d < best_model[0]:
                    best_model = (d, i, pt)

            print best_model[-1]

    def nextFrame(self):
        while True:
            while self.vr.framenum < self.t:
                success, I = self.vr.getNextFrame()

            cv2.imshow('image', I)
            key = cv2.waitKey(0) & 0xFF
            if key == 27:       # esc
                return
            elif key == 32:     # space
                self.t += 10


if __name__ == '__main__':
    ic = ImageClicker()
    ic.nextFrame()
