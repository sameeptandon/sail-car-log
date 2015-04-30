import numpy as np
import os, sys
import glob
import mbly_obj_pb2
import mbly_lane_pb2
# import mbly_ref_pb2
import cv2
from LidarTransforms import R_to_c_from_l

def T_from_mbly_to_lidar(pts, T, R):
    """ Transforms the mobileye output into the lidar's FoR.
        TODO: Add save these values to params
    """
    # R = np.eye(3)
    # pts[:, 0] += 0.762
    # pts[:, 1] += 0.0381
    # pts[:, 2] += -0.9252
    c_pts = pts.copy()
    c_pts[:, :3] += np.tile(T, (pts.shape[0], 1))
    pts_wrt_lidar = np.dot(R, c_pts[:,:3].T).T
    return pts_wrt_lidar

def projectPoints(mbly_data, args, T, R):
    """ Projects mobileye points into the camera's frame
        Args: mbly_data, the output from loadMblyWindow
              args, the output from parse_args
    """
    params = args['params']
    cam_num = args['cam_num']
    cam = params['cam'][cam_num]

    # Move points to the lidar FoR
    pts_wrt_lidar = T_from_mbly_to_lidar(mbly_data, T, R)

    # Move the points to the cam FoR
    pts_wrt_cam = pts_wrt_lidar +\
      cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_cam = np.dot(R_to_c_from_l(cam), pts_wrt_cam.transpose())

    # Project the points into the camera space
    (pix, J) = cv2.projectPoints(pts_wrt_cam.transpose(),
        np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]),
        cam['KK'], cam['distort'])
    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)

    mbly_data_projected = np.hstack((mbly_data, pix.transpose()))
    return mbly_data_projected


class Pb_container(object):
    obj_dict = {'type': mbly_obj_pb2, 'message': 'Objects', 'elem':'object'}
    lane_dict = {'type': mbly_lane_pb2, 'message': 'Lanes', 'elem':'lane'}
    # ref_dict = {'type': mbly_ref_pb2, 'message': 'Ref_pts', 'elem':'ref_pt'}

    def get_pb_dict(self, proto_file_name):
        if '.objproto' in proto_file_name:
            return Pb_container.obj_dict
        elif '.lanesproto' in proto_file_name:
            return Pb_container.lane_dict
        # elif '.refproto' in proto_file_name:
        #     return Pb_container.ref_dict

    def __init__(self, proto_file_name):
        if proto_file_name is None:
            return

        self.proto_file_name = proto_file_name

        self.dict = self.get_pb_dict(proto_file_name)

        type = self.dict['type']
        elem = self.dict['elem']
        message = self.dict['message']

        # Create a generic pb_datum
        # If type = obj -> pb_datum = mbly_obj_pb2.Objects()
        pb_datum = getattr(type, message)()
        with open(self.proto_file_name) as f:
            pb_datum.ParseFromString(f.read())
        # Create a list of all items in the pb_datum
        # If type = obj -> datum = [x for x in pb_datum.Objects().object]
        datum = [x for x in getattr(pb_datum, elem)]

        self.times = set([o.timestamp for o in datum])
        # Times must be sorted for fast lookup
        self.times = np.array(sorted(self.times))

        # Create a mapping from time to objects
        self.datum = {}
        for o in datum:
            if o.timestamp not in self.datum:
                self.datum[o.timestamp] = [o]
            else:
                self.datum[o.timestamp].append(o)

class MblyLoader(object):
    def __init__(self, params):
        obj_proto = params['mbly_obj']
        lane_proto = params['mbly_lanes']
        # ref_proto = params['mbly_ref_pts']
        self.objs = Pb_container(obj_proto)
        self.lanes = Pb_container(lane_proto)
        # self.refs = Pb_container(ref_proto)


    def loadObj(self, microsec_since_epoch):
        return self.loadMblyWindow(microsec_since_epoch, pb_type='objs')

    def loadLane(self, microsec_since_epoch):
        return self.loadMblyWindow(microsec_since_epoch, pb_type='lanes')

    # def loadRef(self, microsec_since_epoch):
    #     return self.loadMblyWindow(microsec_since_epoch, pb_type='refs')

    def loadMblyWindow(self, microsec_since_epoch, pb_type):
        """
        Loads the output of the MobilEye sensor closest to
        microseconds_since_epoch
        pb_type must match an pb_container object belonging to self
        """
        container = getattr(self, pb_type)
        idx = np.searchsorted(container.times, microsec_since_epoch * 1000L,
                              side='right')
        objs = container.datum[container.times[idx]]
        return objs

if __name__ == "__main__":
    loader = MblyLoader(sys.argv[1], sys.argv[2], sys.argv[3])
    print loader.loadObj(loader.objs.times[100] / 1000L)[0]
    print loader.loadLane(loader.objs.times[100] / 1000L)[0]
    for t in loader.obj.times:
        print loader.loadObj(t / 1000L)[0]
