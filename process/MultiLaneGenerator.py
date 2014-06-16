"""
 This module is responsible for outputting multilane boudaries
 output is (x, y, z, lanenum)

"""
import numpy as np
from sklearn import cluster
from scipy.spatial import distance
import random
from scipy.interpolate import UnivariateSpline
import warnings

class MultiLane:
    def __init__(self, filteredLidarFile, singleLaneInterpolationFile, leftnumberlanes, rightnumberlanes):
        self.lanes = np.load(filteredLidarFile)
        self.interp = np.load(singleLaneInterpolationFile)
        self.leftnumberlanes = leftnumberlanes
        self.rightnumberlanes = rightnumberlanes

        self.extendLanes();
        self.filterLaneMarkings();
        self.clusterLanes();
        self.sampleLanes()
        self.interpolateLanes();

    def saveLanes(self, filepath):
        print "Saving Lanes"
        np.save(filepath, self.lanes);

    def clusterLanes(self):
        print "Clustering Lanes"
        lanest = np.array([])
        numsegs = self.lanes.shape[0]/100
        count = 0
        for segment in xrange(numsegs):
            if segment%(numsegs/10) == 0:
                print "Segment ", segment, "/", numsegs
            lanes = self.lanes[(segment*1.0/numsegs)*self.lanes.shape[0]:((segment+1)*1.0/numsegs)*self.lanes.shape[0],:]
            cl = cluster.DBSCAN(eps = 1.5, min_samples = 10)
            labels = cl.fit_predict(lanes[:, :3])
            lanes  = lanes[labels > -1, :]
            labels  = labels[labels > -1]
            labels = labels + count
            count += (np.unique(labels)).shape[0]
            lanes = np.column_stack((lanes, labels))
            if lanest.shape[0]== 0:
                lanest = lanes
            else:
                lanest = np.vstack((lanest, lanes))
        self.lanes = lanest

    def extendLanes(self):
        print "Extending Lanes"
        left = self.interp['left']
        right = self.interp['right']
        lanes = None
        for imod in xrange(self.rightnumberlanes + self.leftnumberlanes):
            i = imod - self.leftnumberlanes
            lanenew = (right - left)*i + right
            if lanes is not None:
                lanes = np.dstack((lanes,lanenew))
            else:
                lanes = lanenew
        self.interp = lanes

    def filterLaneMarkings(self):
        print "Filtering Markings Close to Translated Interpolated Lane Points"
        lastIndex = 0
        numsegs = 200
        multilanedists = np.ones((self.interp.shape[2], self.lanes.shape[0]))*1000000
        #find the current index somehow
        for segment in xrange(numsegs):
            if segment%(numsegs/10) == 0:
                print "Segment ", segment, "/", numsegs
            interpSeg = self.interp[(segment*1.0/numsegs)*self.interp.shape[0]:((segment+1)*1.0/numsegs)*self.interp.shape[0], :, :]
            lastInterpSegPoint = np.array([interpSeg[interpSeg.shape[0]-1, :, 0]]);
            currentIndex= lastIndex + np.argmin(distance.cdist(lastInterpSegPoint, self.lanes[lastIndex:, :3]));
            laneSeg = self.lanes[lastIndex:currentIndex, :];
            for i in xrange(self.interp.shape[2]):
                single = interpSeg[:, :, i]
                singlelanedists = np.min(distance.cdist(laneSeg[:, :3], single[:, :3]), axis =1)
                multilanedists[i, lastIndex:currentIndex] = singlelanedists
            lastIndex = currentIndex
        mins = np.min(multilanedists, axis = 0)
        argmins = np.argmin(multilanedists, axis = 0)
        mask = (((argmins == 0) & (mins <0.20)) | ((argmins == 1) & (mins < 0.8)) | ((argmins == 2) & (mins <1.2)))
        filtered = self.lanes[mask]
        filteredargmins = argmins[mask]
        filtered = np.column_stack((filtered, filteredargmins))
        self.lanes = filtered

    def interpolateLanes(self):
        print "Interpolating Markings"
        total = np.array([])
        for i in xrange(self.leftnumberlanes + self.rightnumberlanes):
            lane = self.lanes[self.lanes[:,self.lanes.shape[1]-2] == i, :]
            lanedist  = np.sum(np.abs(lane[:, :3])**2,axis=-1)**(1./2)
            xinter = UnivariateSpline(lanedist, lane[:,0], s=100)
            yinter = UnivariateSpline(lanedist, lane[:,1], s=100)
            zinter = UnivariateSpline(lanedist, lane[:,2], s=100)
            newpoints = np.arange(0, lanedist[lanedist.shape[0]-1], 0.01)
            a =  np.column_stack((xinter(newpoints), yinter(newpoints), zinter(newpoints), np.ones(newpoints.shape[0])*i))
            if total.shape[0] == 0:
                total = a
            else:
                total = np.vstack((total, a))
        self.lanes = total

    def sampleLanes(self):
        print "Sampling Lanes"
        centroids = []
        for i in np.unique(self.lanes[:,self.lanes.shape[1]-1]):
            sample = random.choice(self.lanes[self.lanes[:,self.lanes.shape[1]-1] == i]);
            centroids.append(sample)
        centroids = np.array(centroids);
        self.lanes = centroids

if __name__ == '__main__':
    warnings.filterwarnings("ignore") #filtering warnings
    multiLane = MultiLane('/Users/Phoenix/pranav/files/multilanefiles/lanesraw2.npy', '/Users/Phoenix/pranav/files/singlelanefiles/interpolated.pickle', 1, 2)
    multiLane.saveLanes('/Users/Phoenix/pranav/files/multilanefiles/lanesInterpolated3.npy')
   
