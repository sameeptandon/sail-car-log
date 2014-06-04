import numpy as np
import cv,cv2
import pickle
from transformations import euler_matrix

blue = np.array([255,0,0])
green = np.array([0,255,0])
red = np.array([0,0,255])
dist = np.array([8,16,24,32,40,48,56,64,72,80])+4

def GetQ50CameraParams():
    cam = [{}, {}]
    for i in [1, 0]:
        cam[i]['R_to_c_from_i'] = np.array([[-1, 0, 0],[0, 0, -1],[0, -1, 0]])

        if i == 0: # left camera

            cam[i]['R_to_c_from_l_in_camera_frame'] = cam[1]['R_to_c_from_l_in_camera_frame']
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = cam[1]['displacement_from_l_to_c_in_lidar_frame']

            # extrinsics parameters for transforming points in right camera frame to this camera
            T = np.array([-0.5873763710461054, 0.00012196510170337307, 0.08922401781210791])
            R = np.array([0.9999355343485463, -0.00932576944123699, 0.006477435558612815, 0.009223923954826548, 0.9998360945238545, 0.015578938158992275, -0.006621659456863392, -0.01551818647957998, 0.9998576596268203])
            R = R.reshape((3,3))
            T = T.reshape((3,1))
            E = np.hstack((R.transpose(), np.dot(-R.transpose(),T)))
            E = np.vstack((E,np.array([0,0,0,1])))
            cam[i]['E'] = E


            cam[i]['fx'] = 2254.76
            cam[i]['fy'] = 2266.30
            cam[i]['cu'] = 655.55
            cam[i]['cv'] = 488.85
            cam[i]['distort'] = np.array([-0.22146000368016028, 0.7987879799679538, -6.542034918087567e-05, 2.8680581938024014e-05, 0.0])

        elif i == 1: # right camera
            R_to_c_from_l_in_camera_frame = euler_matrix(0.044,0.0291,0.0115)[0:3,0:3]
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = np.array([-0.5,0.31,0.34]);
            cam[i]['E'] = np.eye(4)

            cam[i]['fx'] = 2250.72
            cam[i]['fy'] = 2263.75
            cam[i]['cu'] = 648.95
            cam[i]['cv'] = 450.24
            cam[i]['distort'] = np.array([-0.16879238412882028, 0.11971166628565273, -0.0017457365846050555, 0.0001853749033525837, 0.0])
      
        cam[i]['KK'] = np.array([[cam[i]['fx'], 0.0, cam[i]['cu']],
                              [0.0, cam[i]['fy'], cam[i]['cv']],
                              [0.0, 0.0, 1.0]])
        cam[i]['f'] = (cam[i]['fx'] + cam[i]['fy']) / 2
    return cam



cam = GetQ50CameraParams()[0] 


def recover3d(pixels, Z=np.array([10,20,30,40,50,60,70,80])):
  KK = cam['KK']
  N = pixels.shape[0]
  assert(pixels.shape[1] == 2)
  # compute X/Z and Y/Z using inv(KK*R)*pixels
  Pos_S = np.linalg.solve(KK, np.concatenate((pixels,np.ones([N,1])), axis=1).transpose()).transpose()
  Y = Pos_S[:,1]*Z
  X = Pos_S[:,0]*Z
  return np.vstack((X,Y,Z)).transpose()



def get_depth_fontsize(depth, min_font=6, max_font=18):
    min_depth, max_depth = 10, 100
    frac = 1-min((max(depth, min_depth)-min_depth)/float(max_depth-min_depth),1)
    fontsize = (max_font - min_font) * frac + min_font
    return fontsize


def dist2color(dist, max_dist = 100.0):
  # given a distance and a maximum distance, gives a color code for the distance.
  # red being closest, green is mid-range, blue being furthest
  alpha = (dist/max_dist)
  if alpha<0.5:
    color = red*(1-alpha*2)+green*alpha*2
  else:
    beta = alpha-0.5
    color = green*(1-beta*2)+blue*beta*2
  return color.astype(np.int)

def colorful_line(img, start, end, start_color, end_color, thickness):
  # similar to cv.line, but draws a line with gradually (linearly) changing color. 
  # allows starting and ending color to be specified. 
  # implemented using recursion.
  if ((start[0]-end[0])**2 + (start[1]-end[1])**2)**0.5<=thickness*2:
    cv2.line(img, start, end ,start_color,thickness)
    return img
  mid = (int((start[0]+end[0])/2),int((start[1]+end[1])/2))
  mid_color = [int((start_color[0]+end_color[0])/2),int((start_color[1]+end_color[1])/2),int((start_color[2]+end_color[2]))/2]
  img = colorful_line(img, start, mid, start_color, mid_color, thickness)
  img = colorful_line(img, mid, end, mid_color, end_color, thickness)
  return img

def pix2depth(bbox, lane_pred_2d):
  # using lane prediction results, infer the depth of a car using the bottom of its bbox
  top_left = bbox[0]
  bottom_right = bbox[1]
  x = (top_left[0]+bottom_right[0])/2.0
  y = bottom_right[1]
  # find the closest point on lane marking predictions
  sqr_pix_dist = (lane_pred_2d[0,:,:]-x)**2+(lane_pred_2d[1,:,:]-y)**2
  indices = np.argmin(sqr_pix_dist, axis=1)
  # decide whether to use left or right lane markings
  if sqr_pix_dist[0,indices[0]]<sqr_pix_dist[1,indices[1]]:
    lr = 0
  else:
    lr = 1
  idx = indices[lr]
  # decide if the bbox is further or closer than the reference point
  if y>lane_pred_2d[1,lr,idx]:
    idx2=idx-1
  else:
    idx2 = idx+1
  f = (lane_pred_2d[1,lr,idx]-lane_pred_2d[1,lr,idx2])*dist[idx2]*dist[idx]/(dist[idx2]-dist[idx])
  v = lane_pred_2d[1,lr,idx]-f/dist[idx]
  depth = f/(y-v)
  return depth


def draw2D(img, lane_pred_2d, car_pred):
  # draw lane predictions on image
  for j in xrange(lane_pred_2d.shape[2]): # iterate over points on track                          
    lcol = int(round(lane_pred_2d[0,0,j]))
    lrow = int(round(lane_pred_2d[1,0,j]))
    rcol = int(round(lane_pred_2d[0,1,j]))
    rrow = int(round(lane_pred_2d[1,1,j]))
    color =dist2color(dist[j])
    img[lrow-5:lrow+6,lcol-5:lcol+6,:] = color
    img[rrow-5:rrow+6,rcol-5:rcol+6,:] = color
    if j>0:
      img = colorful_line(img, (lcol, lrow) , (prev_lcol, prev_lrow) ,color.tolist(), prev_color.tolist(),4)
      img = colorful_line(img, (rcol, rrow) , (prev_rcol, prev_rrow) ,color.tolist(), prev_color.tolist(),4)
    prev_lcol = lcol
    prev_lrow = lrow
    prev_rcol = rcol
    prev_rrow = rrow
    prev_color = color
  # draw car predictions on image
  for bb in car_pred:
    top_left = tuple(bb['rect'].pos)
    bottom_right = tuple( bb['rect'].pos + bb['rect'].dim )
    if bottom_right[1]<np.min(lane_pred_2d[1,:,0]) and bottom_right[1]>np.max(lane_pred_2d[1,:,-1]):
      depth = pix2depth((top_left, bottom_right), lane_pred_2d)
    else:
      depth = bb['depth']+2
    color = dist2color(depth)
    cv2.rectangle(img, top_left, bottom_right, color.tolist(), 4)
    
    text = '%0.1f' % depth
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = get_depth_fontsize(depth, min_font=0.2, max_font=2.0)
    org = (bb['rect'].pos[0]+30*fontScale, bb['rect'].pos[1]-4*fontScale)
    thickness = 2
    cv2.putText(img, text, org, font, fontScale, color.tolist(), thickness, cv2.CV_AA)
    

    corners = np.zeros([2,2])
    corners[0,:] = np.array(bb['rect'].pos)
    corners[1,:] = np.array([corners[0,0], corners[0,1]+bb['rect'].dim[1]])
    corners3d = recover3d(corners, Z = np.array([depth, depth]))
    car_height = corners3d[1,1]-corners3d[0,1]
    if car_height>2.2:
      car_type = 'Truck'
      type_color = red
    else:
      car_type = 'Car'
      type_color = blue+red
    org = (bb['rect'].pos[0]+5, bb['rect'].pos[1]+bb['rect'].dim[1]/2)
    cv2.putText(img, car_type, org, font, fontScale, type_color.tolist(), thickness, cv2.CV_AA)
  return img


def draw3D(lane_pred_3d, lane_pred_2d, car_pred):
  w =960
  topDown = np.zeros([w,w,3], dtype=np.uint8)                                                 
  font = cv2.FONT_HERSHEY_SIMPLEX
  fontScale = 1.0
  marking_dist = np.array([20, 40, 60, 80])
  marking_y = (marking_dist-9.8)*12
  for i in xrange(marking_dist.shape[0]):
    color = dist2color(marking_dist[i])
    cv2.line(topDown, (0,w-marking_y[i]), (w-1,w-marking_y[i]) , color.tolist(),2)
    cv2.putText(topDown, str(marking_dist[i])+'m', (0,w-marking_y[i]), font, fontScale, color.tolist(), 2, cv2.CV_AA)
  # draw top down lane predictions 
  for j in xrange(lane_pred_3d.shape[2]): # iterate over points on lane markings
    lX = lane_pred_3d[0,0,j]
    lZ = lane_pred_3d[2,0,j]
    rX = lane_pred_3d[0,1,j]
    rZ = lane_pred_3d[2,1,j]

    ly = round(np.clip(w-1-(lZ-4)*12+6+60, 4,w-6))
    lx = round(np.clip(lX*12+w/2+4, 4,w-6))
    ry = round(np.clip(w-1-(rZ-4)*12+6+60, 4,w-6))
    rx = round(np.clip(rX*12+w/2+4, 4,w-6))
    color =dist2color(dist[j])
    topDown[ly-4:ly+5, lx-4:lx+5,:]= color
    topDown[ry-4:ry+5, rx-4:rx+5,:]= color
    if j>0: # draw lines to join points
      topDown = colorful_line(topDown, (lx, ly),(prev_lx,prev_ly) ,color.tolist(), prev_color.tolist(),3)
      topDown = colorful_line(topDown, (rx, ry),(prev_rx,prev_ry) ,color.tolist(), prev_color.tolist(),3)
    prev_lx = lx
    prev_ly = ly
    prev_rx = rx
    prev_ry = ry
    prev_color = color
  # draw top down car predictions
  for bb in car_pred:
    top_left = tuple(bb['rect'].pos)
    bottom_right = tuple( bb['rect'].pos + bb['rect'].dim )
    if bottom_right[1]<np.min(lane_pred_2d[1,:,0]) and bottom_right[1]>np.max(lane_pred_2d[1,:,-1]):
      depth = pix2depth((top_left, bottom_right), lane_pred_2d)
    else:
      depth = bb['depth']+2
    color = dist2color(depth)
    org = (bb['rect'].pos[0]+30*fontScale, bb['rect'].pos[1]-4*fontScale)
    thickness = 2

    corners = np.zeros([4,2])
    corners[0,:] = np.array(bb['rect'].pos)
    corners[1,:] = np.array([corners[0,0], corners[0,1]+bb['rect'].dim[1]])
    corners[2,:] = np.array([corners[0,0]+bb['rect'].dim[0], corners[0,1]])
    corners[3,:] = np.array([corners[0,0]+bb['rect'].dim[0], corners[0,1]+bb['rect'].dim[1]])
    corners3d = recover3d(corners, Z = np.array([depth, depth, depth, depth]))
    corners3d = np.dot(np.transpose(cam['R_to_c_from_l_in_camera_frame']), corners3d.transpose()).transpose()
    car_height = corners3d[1,1]-corners3d[0,1]
    if car_height>2.2:
      car_length = 80
      car_width = 30
      type_color = red
    else:
      car_length = 50
      car_width = 20
      type_color = blue+red
    bly = round(np.clip(w-1-(corners3d[1,2]-4)*12+6+60, 4,w-6))
    blx = round(np.clip(corners3d[1,0]*12+w/2+4, 4,w-6))
    cv2.rectangle(topDown, (blx, bly-car_length), (blx+car_width, bly), type_color.tolist(), -1)
    #org = (bb['rect'].pos[0]+5, bb['rect'].pos[1]+bb['rect'].dim[1]/2)
    #cv2.putText(img, car_type, org, font, fontScale, type_color.tolist(), thickness, cv2.CV_AA)
  return topDown


#fname = 'split_0_to_gilroy_d1'
fname = 'split_0_from_gilroy_b1'
video_name = '/scail/group/deeplearning/driving_data/q50_data/4-3-14-gilroy/'+fname+'.avi'
capture = cv2.VideoCapture(video_name)

car_pred = pickle.load(open('/scail/group/deeplearning/driving_data/brodyh/driving_demo/4-3-14-gilroy/'+fname+'.pkl','r'))
lane_pred_3d = pickle.load(open('/scail/group/deeplearning/driving_data/twangcat/'+fname+'_lane3D.pickle','r'))
lane_pred_2d = pickle.load(open('/scail/group/deeplearning/driving_data/twangcat/'+fname+'_lane2D.pickle','r'))

start_frame = 1800#2380#1650#390-1250
success = True
fCnt = 0
lane_pred_2d*=2
step = 1
while success and fCnt<2600:#lane_pred_3d.shape[3]:
  for i in range(step):
    success, img = capture.read()
  if fCnt<start_frame:
    fCnt+=step
    continue
  img2d = draw2D(img, lane_pred_2d[:,:,:,fCnt], car_pred[fCnt])
  img3d = draw3D(lane_pred_3d[:,:,:,fCnt],lane_pred_2d[:,:,:,fCnt], car_pred[fCnt])



  #cv2.imshow("video", cv2.resize(np.concatenate((img2d, img3d), axis=1), (1120,480)))
  #key = cv2.waitKey(2)
  cv2.imwrite('/scail/group/deeplearning/driving_data/twangcat/pitchdeck/'+str(fCnt-start_frame+1)+'.png',np.concatenate((img2d, img3d), axis=1))
  fCnt+=step
  print fCnt

print 'Finished reading video %s!'%video_name
