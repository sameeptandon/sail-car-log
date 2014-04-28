import cv
import sys, os
from AnnotationLib import *
process_dir = '/'.join(os.path.abspath('show.py').split('/')[:-2])+'/process/';
sys.path.append(process_dir);
from Q50_config import *
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from RadarTransforms import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *
from ProjectRadarOnVideo import *

from collections import defaultdict
from collections import Counter

def set_ids(annotations):                       
        n = len(annotations)            
        available_id = 0;                       
        annotation = annotations[0];
        rects = annotation.rects;
        for rect in rects:      
                rect.classID = available_id; 
                available_id += 1;
        for anno_idx in range(1,n):
                new_annotation = annotations[anno_idx];
                new_rects = new_annotation.rects;
                dist_mat = [[0 for i in range(len(rects))] for j in range(len(new_rects))]
                for i, new_rect in enumerate(new_rects):
                        for j, rect in enumerate(rects):
                                dist_mat[i][j] = (new_rect.centerX()-rect.centerX())**2 + (new_rect.centerY()-rect.centerY())**2 \
                                                 + (new_rect.width()-rect.width())**2 + (new_rect.height()-rect.height())**2
                min_dist = [min(dist_mat[i]) for i in range(len(new_rects))];
                new_ids = [dist_mat[i].index(min(dist_mat[i])) for i in range(len(new_rects))];
                for i in range(len(new_rects)):
                        for j in range(i+1, len(new_rects)):
                                if new_ids[i] == new_ids[j]:
                                        if min_dist[i] < min_dist[j]:
                                                new_ids[j] = -1;
                                        else:
                                                new_ids[i] = -1;
                for i in range(len(new_rects)):
                        if new_ids[i] == -1:
                                new_rects[i].classID = available_id;
                                available_id += 1;
                        else:
                                new_rects[i].classID = rects[new_ids[i]].classID;
                                         
                annotation = new_annotation;
                rects = annotation.rects;
        
        return available_id; 
    

def radarTrackerMapping(annotation, rdr_map, args, sc, l): # l is the length of consistency in annotation's ids
	anno_len = len(annotation);
	rdr_len = len(rdr_map);
	max_id = 1000;
	i = 0;
	
	while (i < anno_len and i < rdr_map):
		id_info = [[] for r in range(max_id)];
		eq = 1;
		for j in range(l):
			idx = i + j;
			if idx >=len(annotations):
				break;
			annotation = annotations[idx];
			frame_num = sc * idx;
			radar_data = loadRDR(rdr_map[frame_num])[0]
			if radar_data.shape[0] > 0:
			    # Remove points that have a low radar cross-section
			    mask = (radar_data[:, 5] > 0)
			    # Remove points that are moving too fast (fixed objects)
			    mask &= (radar_data[:, 6] > -20)
			    radar_data = radar_data[mask]
			
		        center_proj = projectPoints(np.array(radar_data), args)
		        cn_rdr = center_proj[:, 7:10].astype(np.int32);
			cn_trk = np.array([[eq*rect.classID, int(rect.centerX()), int(rect.bottom())] 
					for rect in annotation.rects if rect.width() > 40]);
			print cn_rdr;
			print cn_trk;
			for r in range(len(cn_trk)):
				id_r = cn_rdr[0,0];
				dis = (cn_rdr[0,1]-cn_trk[r,1])**2 + (cn_rdr[0,2]-cn_trk[r,2])**2;
				for s in range(1,len(cn_rdr)):
					dis_new = (cn_rdr[s,1]-cn_trk[r,1])**2 + (cn_rdr[s,2]-cn_trk[r,2])**2;
					if dis_new < dis:
						dis = dis_new;
						id_r = cn_rdr[s,0];

				id_info[cn_trk[r,0] + max_id/2].append([id_r, dis]);
		max_ID = [[0,0] for r in range(max_id)];
		for r in range(max_id):
			arr = np.array(id_info[r]);
			if len(arr) > 0:
				result = Counter(arr[:,0]).most_common(1)[0]
	#			d = defaultdict(int)
	#			for s in arr:
	#			    d[s] += 1
	#			result = max(d.iteritems(), key=itemgetter(1));
				if result[1] > .5 * len(arr):
					max_ID[r][0] = result[0];
					max_ID[r][1] = sum([arr[sx][1] for sx in range(len(arr)) if arr[sx][0]==result[0]])
				else:
					max_ID[r][0] = -1;
#		for r in range(max_id):
#			for s in range(r,max_id):
#				if max_ID[r][0]==max_ID[s][0] and max_ID[r][0]!=-1:
#					if max_ID[r][1] < max_ID[s][1]:
#						max_ID[s][0] = -1;
#					else:
#						max_ID[r][0] = -1;
				
		print id_info;
		print max_ID; 
		for j in range(l):
			idx = i + j;
			if idx >= len(annotations):
				break;
			annotation = annotations[idx]
			frame_num = sc * idx;
			radar_data = loadRDR(rdr_map[frame_num])[0]
			if radar_data.shape[0] > 0:
			    # Remove points that have a low radar cross-section
			    mask = (radar_data[:, 5] > 0)
			    # Remove points that are moving too fast (fixed objects)
			    mask &= (radar_data[:, 6] > -20)
			    radar_data = radar_data[mask]
			
			center_proj = projectPoints(np.array(radar_data), args)
			cn_rdr = center_proj[:, 7:10].astype(np.int32);
			
			new_ids = [max_ID[rect.classID + max_id/2][0] for rect in annotation.rects];
#			print "new_ids: ", new_ids
			for r in range(len(new_ids)):
				for s in range(r+1,len(new_ids)):
					if new_ids[r] == new_ids[s] and new_ids[r] > 0:
						cn_idx = 0;
						flag = False;
#						print r,s, len(cn_rdr), "============="
						while(True):
							print cn_idx
							if cn_rdr[cn_idx][0] == new_ids[r]:
								flag = True;
								break;
							cn_idx += 1;
							if cn_idx > len(cn_rdr)-1:
								break;
						if flag:
						 
							r_pnt = [int(annotation.rects[r].centerX()), int(annotation.rects[r].bottom())]	
							s_pnt = [int(annotation.rects[s].centerX()), int(annotation.rects[s].bottom())]	
							dis_r = (cn_rdr[cn_idx,1]-r_pnt[0])**2 + (cn_rdr[cn_idx,2]-r_pnt[1])**2;
							dis_s = (cn_rdr[cn_idx,1]-s_pnt[0])**2 + (cn_rdr[cn_idx,2]-s_pnt[1])**2;
							if(dis_r < dis_s):
								new_ids[s] = -1;
							else:
								new_ids[r] = -1;
						
		        for r, rect in enumerate(annotation.rects):
				r_id = new_ids[r];
				rect.score = -1;
				if r_id == -1:
					rect.classID = -abs(rect.classID);
				else:
					rect.classID = r_id;
					cn_idx = 0;
					flag = False;
					while(True):
						print cn_idx;
						if radar_data[cn_idx][7] == r_id:
							flag = True;
							break;
						cn_idx += 1;
						if cn_idx >= len(radar_data):
							break;
					if flag:
						rect.score = radar_data[cn_idx][0];
		i += l;

if __name__ == "__main__":
	if (len(sys.argv) < 5):
		print "python show.py <.al file> <frame#> <directory of .avi> <video file>"
		sys.exit();
	
	writer = cv2.VideoWriter('edited_sequence_test3D.avi', cv.CV_FOURCC('X','V', 'I', 'D'),
                    20.0, (1280,960) )	
	filename = (sys.argv[1]);
	annotations = parseXML(filename);
	set_ids(annotations);
	print len(annotations);
	di = 1;
	ind = int(sys.argv[2]);


####
	args = parse_args(sys.argv[3], sys.argv[4])
  	params = args['params']
   
  	video_reader = VideoReader(args['video'])
   	rdr_map = loadRDRCamMap(args['map'])
	
	radarTrackerMapping(annotations, rdr_map, args, 10, 20);   
	
	saveXML(filename.split('.')[0] + "_with_distance.al", annotations);
	while True:
	
#		choice = raw_input("> ");
		
		choice = 'f';
		if choice=='f': di = 1;
		elif choice=='b': di = -1;
		elif choice == 'x': break;
		annotation = annotations[ind];	
		ImgName =  annotation.filename();
		ImgName = "/Users/Carrie/Desktop/radarData/all_extracted/"+'/'.join(ImgName.split('/')[-2:])
		print ImgName;
#		ImgName = "/Users/Carrie/Desktop/radarData/"+ImgName[14:]	
		
		frame_num = ind * 10
		radar_data = loadRDR(rdr_map[frame_num])[0]
		 
		print ImgName;
		print os.path.isfile(ImgName)
		rects = annotation.rects;
		I = cv2.imread(ImgName);
#		cv2.imshow('img1',I);
#		cv2.waitKey(0)
		bottom_right_pnt = [];
		
		for i, rect in enumerate(rects):
			bottom_right_pnt.append([int(rect.classID), int(rect.x2), int(rect.y2)]);
			

		if radar_data.shape[0] > 0:
		    # Remove points that have a low radar cross-section
		    mask = (radar_data[:, 5] > 0)
		    # Remove points that are moving too fast (fixed objects)
		    mask &= (radar_data[:, 6] > -20)
		    radar_data = radar_data[mask]

		if radar_data.shape[0] > 0:
		    front_right_pts = np.array(radar_data)
		    front_right_pts[:,0] += radar_data[:,3]
		    front_right_pts[:,1] += radar_data[:,4] / 2.

		    front_left_pts = np.array(radar_data)
		    front_left_pts[:,0] += radar_data[:,3]
		    front_left_pts[:,1] -= radar_data[:,4] / 2.

		    right_pts = np.array(radar_data)
		    right_pts[:,1] += radar_data[:,4] / 2.

		    left_pts = np.array(radar_data)
		    left_pts[:,1] -= radar_data[:,4] / 2.

		    # reproject camera_t points in camera frame
		    front_right_proj = projectPoints(front_right_pts, args)
		    front_left_proj = projectPoints(front_left_pts, args)
		    left_proj = projectPoints(left_pts, args)
		    right_proj = projectPoints(right_pts, args)
		    center_proj = projectPoints(np.array(radar_data), args)
		    for r, rect in enumerate(rects):
				cn_idx = 0;
				flag = False;
				while(True):
					if left_proj[cn_idx][7] == rect.classID:
						flag = True;
						break;
					cn_idx += 1;
					if cn_idx > len(left_proj)-1:
						break;
				if not flag:
					cv2.rectangle(I, (int(rect.x1),int(rect.y1)), (int(rect.x2), int(rect.y2)), (0,0,255),2);
				else:
					radar_width_1 = front_right_proj[cn_idx][8]-left_proj[cn_idx][8];
					radar_width_2 = front_left_proj[cn_idx][8]-right_proj[cn_idx][8];
					radar_width = max(abs(radar_width_1), abs(radar_width_2));
					if radar_width < 4: 
						cv2.rectangle(I, (int(rect.x1),int(rect.y1)), (int(rect.x2), int(rect.y2)), (0,0,255),2);
						break;
					else:
						scale_factor = rect.width()/radar_width;
						#scale_factor = 1;
						print scale_factor;
						flag0 = False;
						#cv2.circle(I, tuple(front_left_proj[cn_idx][8:10].astype(np.int32)), 4, (255,255,0))
						if front_right_proj[cn_idx][8] > right_proj[cn_idx][8]:
							flag0 = True;
						#	cv2.circle(I, tuple(left_proj[cn_idx][8:10].astype(np.int32)), 4, (255,255,0))
							pnt0 = [int(rect.x1), int(rect.y2)];
							#print right_proj[cn_idx][8:10], left_proj[cn_idx][8:10]
							r_vector = [int(scale_factor * (left_proj[cn_idx][8]-right_proj[cn_idx][8])), 0];
							f_vector = [int(scale_factor * (front_left_proj[cn_idx][8]-left_proj[cn_idx][8])), 
									int(scale_factor * (front_left_proj[cn_idx][9] - left_proj[cn_idx][9]))]
							print rect.classID, r_vector, f_vector,  rect.width();
							t_vector = [0, -int(rect.height())];
							
							bbl = pnt0;
							bbr = [pnt0[0] + r_vector[0], pnt0[1]];
							bfl = [pnt0[0] + f_vector[0], pnt0[1] + f_vector[1]];
							bfr = [pnt0[0] + f_vector[0] + r_vector[0], pnt0[1] + f_vector[1]];
							tbl = [pnt0[0], pnt0[1] + t_vector[1]];
							tbr = [pnt0[0] + r_vector[0], pnt0[1] + t_vector[1]];
							tfl = [pnt0[0] + f_vector[0], pnt0[1] + int(-.2*f_vector[1]) + t_vector[1]];
							tfr = [pnt0[0] + f_vector[0] + r_vector[0], pnt0[1] + int(-.2*f_vector[1]) + t_vector[1]];
						else:
							r_vector = [int(scale_factor * (left_proj[cn_idx][8]-right_proj[cn_idx][8])), 0];
							f_vector = [int(scale_factor * (front_right_proj[cn_idx][8]-right_proj[cn_idx][8])), 
									int(scale_factor * (front_right_proj[cn_idx][9] - right_proj[cn_idx][9]))]
							t_vector = [0, -int(rect.height())];
							pnt0 = [int(rect.x2)- r_vector[0] , int(rect.y2)];
							
							bbl = pnt0;
							bbr = [pnt0[0] + r_vector[0], pnt0[1]];
							bfl = [pnt0[0] + f_vector[0], pnt0[1] + f_vector[1]];
							bfr = [pnt0[0] + f_vector[0] + r_vector[0], pnt0[1] + f_vector[1]];
							tbl = [pnt0[0], pnt0[1] + t_vector[1]];
							tbr = [pnt0[0] + r_vector[0], pnt0[1] + t_vector[1]];
							tfl = [pnt0[0] + f_vector[0], pnt0[1] + int(-.2*f_vector[1]) + t_vector[1]];
							tfr = [pnt0[0] + f_vector[0] + r_vector[0], pnt0[1] + int(-.2*f_vector[1]) + t_vector[1]];
							 
							
				#		cv2.line(I, tuple(bfr), tuple(bfl), (255,255,0))
						if not flag0:
							cv2.line(I, tuple(bfl), tuple(bbl), (255,255,0))
						cv2.line(I, tuple(bbl), tuple(bbr), (255,0,0))
						if flag0: 
							cv2.line(I, tuple(bbr), tuple(bfr), (255,255,0))
					
						cv2.line(I, tuple(tfr), tuple(tfl), (255,255,0),1)
						cv2.line(I, tuple(tfl), tuple(tbl), (255,255,0),1)
						cv2.line(I, tuple(tbl), tuple(tbr), (255,0,0),2)
						cv2.line(I, tuple(tbr), tuple(tfr), (255,255,0),1)
						if flag0:
							cv2.line(I, tuple(tfr), tuple(bfr), (255,255,0))
						if not flag0:
							cv2.line(I, tuple(tfl), tuple(bfl), (255,255,0))
						cv2.line(I, tuple(tbl), tuple(bbl), (255,0,0),2)
						cv2.line(I, tuple(tbr), tuple(bbr), (255,0,0),2)
						
						cv2.putText(I, str(rect.score).split('.')[0], tuple(tbr),
						    cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255), thickness=1)
	#	    mmap = RadarTrackerMapping(right_proj[:,8:10], np.array(bottom_right_pnt));
		    
		    for j in xrange(front_right_proj.shape[0]):
			fr = front_right_proj[j, 8:10].astype(np.int32);
			fl = front_left_proj[j, 8:10].astype(np.int32);
			bl = left_proj[j, 8:10].astype(np.int32);
			br = right_proj[j, 8:10].astype(np.int32);
			cn = center_proj[j, 8:10].astype(np.int32);
			
#			cv2.circle(I, tuple(cn), 4, (255,255,0))
#			cv2.line(I, tuple(fr), tuple(fl), (255,255,0))
#			cv2.line(I, tuple(fl), tuple(bl), (255,255,0))
#			cv2.line(I, tuple(bl), tuple(br), (255,0,0))
#			cv2.line(I, tuple(br), tuple(fr), (255,255,0))

			dist = radar_data[j, 0]
			rcs = radar_data[j, 5]
			spd = radar_data[j, 6]
			id = int(radar_data[j, 7])
			s = "%d: %d, %0.2f, %0.2f" % (id, rcs, dist, spd)
			s = "%d" %(id);
#			cv2.putText(I, s, tuple(fr),
#			    cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255), thickness=1)
	
#		for rect in rects:
#			Is = cv2.rectangle(I, (int(rect.x1),int(rect.y1)), (int(rect.x2), int(rect.y2)), 255,2);
#			cv2.putText(I, str(rect.classID), (int(rect.x2),int(rect.y1)),
#			    cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255), thickness=1)

		ind += di;
		cv2.imshow('img1',I);
		writer.write(I);
		cv2.waitKey(20)
