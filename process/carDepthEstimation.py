import pickle, glob, sys, sympy
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat, savemat
from CameraReprojection import *
from CameraParams import *
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from WGS84toENU import *

driving_data_dir = '/scail/group/deeplearning/sail-deep-gpu/brodyh/labeled_driving_images/'
driving_images_dir = '/scail/group/deeplearning/driving_data/andriluka/IMAGES/driving_data_twangcat/all_extracted/'
preprocessed_gps_data_dir = '/scail/group/deeplearning/sail-deep-gpu/willsong/driving/'

def get_gps_file_prefix(id):
  prefix = str(id.split('-')[-1])
  splits = prefix.split('_')
  return splits[-3] + '_' + splits[-2][:-1]

def get_id(name):
  if isinstance(name, str):
    # Brody's first set of data
    name = '7-16-sacramento-' + name.replace('_', '-', 1)
  else:
    name = str(name)
  id = '-'.join(name.split('-')[:-1]) + '-' + get_gps_file_prefix(name)
  if name.split('-')[0] in ['north', 'south']:
    # Another special case when the date is ommitted
    id = '7-25-bay-' + id
  return id

def get_folder_name(id):
  splits = id.split('-')[:-1]
  if splits[-2] in ['north', 'south', 'east', 'west']:
    return '-'.join(splits[:-2]) + '/' + splits[-2] + '/' + splits[-1]
  else:
    return '-'.join(splits[:-1]) + '/' + splits[-1]

def get_cam(name):
  return name.split('_')[-2][-1]

def get_frame(name):
  num_splits = 10
  frame_in_split = int(name.split('_')[-1])
  split = int(name.split('_')[-4])
  return (frame_in_split - 1) * num_splits + split

def get_orthogonal_velocity(data):
  # Given the GPS data, for each frame, compute two GPS points such that their line segment is
  # orthogonal to the vecolcity vector at that frame using flat ground assumption.
  points1 = np.zeros((data.shape[0], 4))
  points2 = np.zeros((data.shape[0], 4))
  orthogonals = np.cross(data[:, 4:7], np.array([0,0,1]), axisa=1)
  orthogonals /= np.sqrt((orthogonals** 2).sum(-1))[..., np.newaxis]
  for i in xrange(data.shape[0]):
    points1[i, 1:4] = ENUtoWGS84(data[i, 1:4], np.array([orthogonals[i, :]])).transpose()
    points2[i, 1:4] = ENUtoWGS84(data[i, 1:4], np.array([-orthogonals[i, :]])).transpose()
  return points1, points2

def preprocess_all_gps_data():
  name2bb = pickle.load(open('/scail/group/deeplearning/sail-deep-gpu/imagenet_dicts/driving_name2bb.pkl'))
  lane_prefix = '/scail/group/deeplearning/driving_data/nikhilb/recovered_lane_labels/' + \
      '_scail_group_deeplearning_driving_data_twangcat_raw_data_backup_'
  gps_prefix = '/scail/group/deeplearning/driving_data/twangcat/raw_data_backup/'
  lane_prefix = '/scail/group/deeplearning/driving_data/nikhilb/recovered_lane_labels/' + \
      '_scail_group_deeplearning_driving_data_twangcat_raw_data_backup_'

  gps_data = {}
  for name, bb_info in sorted(name2bb.iteritems()):
    id = get_id(name)
    if id in gps_data:
      continue

    gps_file_prefix = id.split('-')[-1]
    gps_filename = gps_file_prefix + '_gps.out'
    folder_name = get_folder_name(id)
    try:
      gps_reader = GPSReader(gps_prefix + folder_name + '/' + gps_filename)
    except:
      print 'Skipped:', id
      continue
    gps_data[id] = {'gps_data' : gps_reader.getNumericData()}
    gps_data[id]['gps_data'][:, [4, 5]] = gps_data[id]['gps_data'][:, [5, 4]]
    ov1, ov2 = get_orthogonal_velocity(gps_data[id]['gps_data'])

    for camIdx in xrange(1, 3):
      cam = getCameraParams()[camIdx - 1]
      tr = GPSTransforms(gps_data[id]['gps_data'], cam)
      lane_path = lane_prefix + folder_name.replace('/', '_') + '-' + gps_file_prefix + \
          str(camIdx) + '-interpolated.mat'
      try:
        lane_labels = loadmat(lane_path)
      except:
        print 'No lane labels:', lane_path
        continue
      lp = lane_labels['left']
      rp = lane_labels['right']
      Tc = np.eye(4)
      left_XYZ = pixelTo3d(lp, cam)
      right_XYZ = pixelTo3d(rp, cam)

      left_Pos = np.zeros((lp.shape[0], 4))
      right_Pos = np.zeros((rp.shape[0], 4))
      ov1_Pos = np.ones((ov1.shape[0], 4))
      ov2_Pos = np.ones((ov2.shape[0], 4))
      ov1_Pos[:, 0:3] = GPSPos(ov1, cam, gps_data[id]['gps_data'][0, :]).transpose()
      ov2_Pos[:, 0:3] = GPSPos(ov2, cam, gps_data[id]['gps_data'][0, :]).transpose()
      for t in range(min(lp.shape[0], tr.shape[0])):
        left_Pos[t,:] = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([left_XYZ[t,0], left_XYZ[t,1], left_XYZ[t,2], 1])))
        right_Pos[t,:] = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([right_XYZ[t,0], right_XYZ[t,1], right_XYZ[t,2], 1])))

      gps_data[id]['tr' + str(camIdx)] = tr
      gps_data[id]['left_Pos' + str(camIdx)] = left_Pos
      gps_data[id]['right_Pos' + str(camIdx)] = right_Pos
      gps_data[id]['ov1_Pos' + str(camIdx)] = ov1_Pos
      gps_data[id]['ov2_Pos' + str(camIdx)] = ov2_Pos

    print len(gps_data), 'gps files pre-processed'
  pickle.dump(gps_data, open(preprocessed_gps_data_dir + 'preprocessed_gps_data_using_velocity.pkl', 'w'))

def plot_box(xmin, ymin, xmax, ymax, color, linestyle = '-', linewidth = 2.0):
  plt.plot([xmin,xmin],[ymin,ymax],color=color, linestyle=linestyle, linewidth=linewidth)
  plt.plot([xmin,xmax],[ymin,ymin],color=color, linestyle=linestyle, linewidth=linewidth)
  plt.plot([xmin,xmax],[ymax,ymax],color=color, linestyle=linestyle, linewidth=linewidth)
  plt.plot([xmax,xmax],[ymin,ymax],color=color, linestyle=linestyle, linewidth=linewidth)

def get_area(box):
  return (box['xmax'] - box['xmin']) * (box['ymax'] - box['ymin'])

def interpolateXZ(pixels, xyz, i, y):
  # Given the two lane labeling points that enclose the target y, interpolate x and z based on the
  # ratio of the distance from y to those two points.
  assert y >= pixels[1, i + 1]
  if pixels[1, i + 1] == pixels[1, i]:
    return ((pixels[0, i + 1] + pixels[0, i]) / 2, (xyz[2, i] + xyz[2, i + 1]) / 2)
  y = float(y)
  fraction = (y - pixels[1, i + 1]) / (pixels[1, i] - pixels[1, i + 1])
  if fraction > 1:
    print fraction
    fraction = 1
  return (pixels[0, i + 1] * (1 - fraction) + pixels[0, i] * fraction, \
      xyz[2, i + 1] * (1 - fraction) + xyz[2, i] * fraction)

def perp_distance_from_point_to_line(px, py, l1x, l1y, l2x, l2y):
  # Compute the shortest distance from a point to a line, where the line is represented as a line
  # segment
  l1 = sympy.geometry.Point(l1x, l1y)
  l2 = sympy.geometry.Point(l2x, l2y)
  l = sympy.geometry.Line(l1, l2)
  p = sympy.geometry.Point(px, py)
  return l.perpendicular_segment(p).length

def display_bb(name, name2bb, lpix, rpix, ov1pix, ov2pix):
  plt.clf()
  try:
    image_folder_name = str('_'.join(name.split('_')[:-1])) + '/'
    imagename = str(driving_images_dir + image_folder_name + name + '.jpeg')
    im = plt.imread(imagename)
  except:
    imagename = '/scail/group/deeplearning/sail-deep-gpu/brodyh/labeled_driving_images/' + name + '.jpeg'
    im = plt.imread(imagename)

  implot = plt.imshow(im)
  plt.plot(lpix[0, :], lpix[1, :], color = 'b')
  plt.plot(rpix[0, :], rpix[1, :], color = 'b')

  for i in xrange(0, ov1pix.shape[1]):
    plt.plot([ov1pix[0, i], ov2pix[0, i]], [ov1pix[1, i], ov2pix[1, i]], color = 'r', linestyle = '-', linewidth = 1.0)

  for b in name2bb[name]['boxes']:
    if get_area(b) < 20: continue
    plot_box(b['xmin'], b['ymin'], b['xmax'], b['ymax'], 'g')
    if b['depth'] == 0:
      plt.text((b['xmin'] + b['xmax']) / 2, (b['ymax'] + b['ymin']) / 2, 'unknown', color = 'm')
    else:
      plt.text((b['xmin'] + b['xmax']) / 2, (b['ymax'] + b['ymin']) / 2 - 10, str(int(b['depth'])), color = 'm')

  plt.axis('off')
  plt.xlim([1920, 0])
  plt.ylim([1280, 0])
  plt.draw()
  plt.pause(1.5)

def gen_name2bb_with_depths(display = True, useVelocity = True):
  gps_data = pickle.load(open(preprocessed_gps_data_dir + 'small_preprocessed_gps_data_using_velocity.pkl'))
  name2bb = pickle.load(open('/scail/group/deeplearning/sail-deep-gpu/imagenet_dicts/driving_name2bb.pkl'))

  skipped = 0
  for name, bb_info in sorted(name2bb.iteritems()):
    id = get_id(name)

    try:
      data = gps_data[id]
      camStr = get_cam(name)
      start = get_frame(name)
      end = start + 500
      skip = 1
      left_Pos = data['left_Pos' + camStr]
      right_Pos = data['right_Pos' + camStr]
      ov1_Pos = data['ov1_Pos' + camStr]
      ov2_Pos = data['ov2_Pos' + camStr]
      tr = data['tr' + camStr]
    except:
      skipped += 1
      continue

    cam = getCameraParams()[int(camStr) - 1]
    lpts = left_Pos[start:end:skip, :]
    lPos2 = np.linalg.solve(tr[start, :, :], lpts.transpose())
    lpix = np.around(np.dot(cam['KK'], np.divide(lPos2[0:3,:], lPos2[2, :])))
    lpix = lpix.astype(np.int32)

    rpts = right_Pos[start:end:skip, :]
    rPos2 = np.linalg.solve(tr[start, :, :], rpts.transpose())
    rpix = np.around(np.dot(cam['KK'], np.divide(rPos2[0:3,:], rPos2[2, :])))
    rpix = rpix.astype(np.int32)

    ov1pts = ov1_Pos[start:end:skip, :]
    ov1Pos2 = np.linalg.solve(tr[start, :, :], ov1pts.transpose())
    ov1pix = np.around(np.dot(cam['KK'], np.divide(ov1Pos2[0:3,:], ov1Pos2[2, :])))
    ov1pix = ov1pix.astype(np.int32)

    ov2pts = ov2_Pos[start:end:skip, :]
    ov2Pos2 = np.linalg.solve(tr[start, :, :], ov2pts.transpose())
    ov2pix = np.around(np.dot(cam['KK'], np.divide(ov2Pos2[0:3,:], ov2Pos2[2, :])))
    ov2pix = ov2pix.astype(np.int32)

    for b in name2bb[name]['boxes']:
      if get_area(b) < 20: continue
      bottom = b['ymax']
      center = (b['xmax'] + b['xmin']) / 2

      if not useVelocity:
        # Use gps location of lane labels to directly interpret Z with respect to Y.
        results = []
        for i in xrange(lpix.shape[1] - 1):
          if lpix[1, i] >= bottom and lpix[1, i + 1] <= bottom:
            x, z = interpolateXZ(lpix, lPos2, i, bottom)
            results.append((z, abs(center - x)))
            break
        for i in xrange(rpix.shape[1] - 1):
          if rpix[1, i] >= bottom and rpix[1, i + 1] <= bottom:
            x, z = interpolateXZ(rpix, rPos2, i, bottom)
            results.append((z, abs(center - x)))
            break

        if len(results) == 0:
          b['depth'] = 300
        elif len(results) == 1:
          b['depth'] = results[0][0]
        else:
          if results[0][1] < results[1][1]:
            b['depth'] = results[0][0]
          else:
            b['depth'] = results[1][0]
      else:
        # Infer depth using the orthogonal vector that is perpendicular to the velocity

        # Since the computation of all perpendicular vectors is relatively expensive, instead of
        # searching thorugh all vectors, we perform a bidirectional search starting at the location
        # as appriximated by the lane labels, and stop the search when the distance begins to
        # increase consecutively.
        for l in xrange(lpix.shape[1] - 2, 0, -1):
          if lpix[1, l] >= bottom and lpix[1, l + 1] <= bottom:
            break
        for r in xrange(rpix.shape[1] - 2, 0, -1):
          if rpix[1, r] >= bottom and rpix[1, r + 1] <= bottom:
            break

        # m is an estimated frame number that indicates the depth of the car
        m = l + r / 2

        closest_distance = 9e99
        num_getting_further = 0
        num_getting_further_thresh = 3
        depth = 300
        for i in xrange(min(m, ov1pix.shape[1] - 1), ov1pix.shape[1]):
          try:
            d = perp_distance_from_point_to_line(center, bottom, ov1pix[0, i], ov1pix[1, i], ov2pix[0, i], ov2pix[1, i])
          except:
            continue
          if d < closest_distance:
            num_getting_further = 0
            closest_distance = d
            depth = ov1Pos2[2, i]
          else:
            num_getting_further += 1
            if num_getting_further > num_getting_further_thresh:
              break

        num_getting_further = 0

        for i in xrange(min(m, ov1pix.shape[1] - 1), 2, -1):
          try:
            # Get the distance from the bottom center of the bounding box to the line vector
            d = perp_distance_from_point_to_line(center, bottom, ov1pix[0, i], ov1pix[1, i], ov2pix[0, i], ov2pix[1, i])
          except:
            continue
          if d < closest_distance:
            num_getting_further = 0
            closest_distance = d
            depth = ov1Pos2[2, i]
          else:
            num_getting_further += 1
            if num_getting_further > num_getting_further_thresh:
              break

        b['depth'] = depth

    if display:
      display_bb(name, name2bb, lpix, rpix, ov1pix, ov2pix)

  print skipped, 'entriese were skipped'
  if not display:
    pickle.dump(name2bb, open('all_depth_using_velocity.pkl', 'w'))

def main():
  if sys.argv[1] == '-p':
    preprocess_all_gps_data()
  elif sys.argv[1] == '-g':
    gen_name2bb_with_depths(display = len(sys.argv) > 2 and sys.argv[2] == '-d')
  else:
    print 'Error: no option is specified.'
    exit(1)

if __name__ == "__main__":
  main()

