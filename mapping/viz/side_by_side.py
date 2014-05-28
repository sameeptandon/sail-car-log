'''
Visualize videos side by side
'''
import sys
import cv
import cv2
import numpy as np

if __name__ == '__main__':
    video_file1 = sys.argv[1]
    video_file2 = sys.argv[2]
    output_file = sys.argv[3]

    capture1 = cv2.VideoCapture(video_file1)
    capture2 = cv2.VideoCapture(video_file2)

    fc1 = capture1.get(cv.CV_CAP_PROP_FRAME_COUNT)
    fc2 = capture2.get(cv.CV_CAP_PROP_FRAME_COUNT)
    assert fc1 == fc2, 'frame counts unequal, %d != %d' % (fc1, fc2)

    fps = 30  # PARAM
    fourcc = cv2.cv.CV_FOURCC(*'MJPG')
    video_writer = cv2.VideoWriter()
    video_writer.open(output_file, fourcc, fps, (1280, 480))  # PARAM

    while True:
        success1, img1 = capture1.read()
        success2, img2 = capture2.read()
        if not success1 or not success2:
            break
        img1 = cv2.resize(img1, (640, 480))
        img2 = cv2.resize(img2, (640, 480))
        side_by_side = np.zeros((480, 1280, 3), np.uint8)
        side_by_side[:480, :640, :] = img1
        side_by_side[:480, 640:, :] = img2
        video_writer.write(side_by_side)

    video_writer.release()
