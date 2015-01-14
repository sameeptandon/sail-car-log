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

        self.lanes = None
        self.markings = None

    def mouseHandler(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:

            if self.markings == None:
                self.markings = np.array([x, y])[np.newaxis]
            else:
                self.markings = np.vstack([self.markings, [x, y]])

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
            if self.lanes == None:
                self.lanes = best_model[-1]
            else:
                self.lanes = np.vstack((self.lanes, best_model[-1]))

    def exportData(self, file_name):
        lanes = {}
        lanes['lane0'] = self.lanes

        print 'saved:'
        print self.lanes
        np.savez(file_name, **lanes)

    def nextFrame(self):
        while True:
            while self.vr.framenum < self.t:
                success, I = self.vr.getNextFrame()

            if self.markings != None:
                x = self.markings[:, 0]
                y = self.markings[:, 1]
                for i in xrange(4):
                    I[y+i, x, :] = np.array([255, 255, 0])
                    I[y-i, x, :] = np.array([255, 255, 0])
                    I[y, x+i, :] = np.array([255, 255, 0])
                    I[y, x-i, :] = np.array([255, 255, 0])

            cv2.imshow('image', I)
            key = cv2.waitKey(1) & 0xFF

            if key != 255:
                print key
                if key == 27:   # esc
                    return
                if key == 115:  # s
                    self.exportData(sys.argv[1] + '/multilane_points_image.npz')
                elif key == 32: # space
                    self.t += 20
                    self.markings = None


if __name__ == '__main__':
    ic = ImageClicker()
    ic.nextFrame()
