from VideoReader import * 
from ArgParser import *
import sys, pickle
import shutil

EXPORT_DIR = 'with_cars'

def getFrameNumFromImage(img_name):
  num_jpeg = img_name.split('_')[-1]
  num = num_jpeg.split('.')[0]
  return int(num)

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    reader = VideoReader(args['video'])
    fdir = sys.argv[1] + '/car_frames'
    fname = fdir + "/" + args['basename'] + str(args['cam_num']) + '.pickle'
    f = open(fname, 'r')
    framenums = pickle.load(f)

    images_loc = sys.argv[3]
    video_arg = args['basename'] + str(args['cam_num'])
    image_dir = images_loc + '/' + filter(lambda x: video_arg in x, os.listdir(images_loc))[0]

    out_dir = image_dir + '/' + EXPORT_DIR
    try:
      os.mkdir(out_dir)
    except:
      pass

    for img in sorted(filter(lambda x: 'jpeg' in x, os.listdir(image_dir))):
      num = getFrameNumFromImage(img)*10
      if num in framenums:
        shutil.copy(image_dir + '/' + img, out_dir)
        



    

