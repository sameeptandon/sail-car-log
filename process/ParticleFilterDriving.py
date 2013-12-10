import numpy as np
from ParticleFilter import *
from scipy import spatial
from CameraParams import *

class ParticleFilterDriving(ParticleFilter):

	def __init__(self,initState,initVar,numParticles,map,trMap,cam_num):
		ParticleFilter.__init__(self,initState,initVar,numParticles)
		self.map = map
		self.trMap = trMap
		self.cam = getCameraParams()[cam_num-1]
		self.S =  np.insert(np.cumsum(np.sqrt( np.diff(self.map[:,0])**2 + np.diff(self.map[:,1])**2 + np.diff(self.map[:,2])**2)),0,0)
		self.mapInterp = np.zeros((np.floor(self.S[-1]*10),3))
		interpIdx = np.arange(0,np.floor(self.S[-1]*10)/10.0,0.1)
		self.mapInterp[:,0] = np.interp(interpIdx,self.S,self.map[:,0])
		self.mapInterp[:,1] = np.interp(interpIdx,self.S,self.map[:,2])
		self.mapInterp[:,2] = np.interp(interpIdx,self.S,self.map[:,1])
		self.searchTree = spatial.KDTree(self.mapInterp[:,[0,2]])

	def propogate(self,gpsInput):
		dS = np.linalg.norm(gpsInput[0])
		dX = dS*np.cos(gpsInput[1])
		dY = dS*np.sin(gpsInput[1])
		dTheta = gpsInput[2]
		super(ParticleFilterDriving,self).propogate((dX,dY,dTheta),(dX*.1,dY*.1,dTheta*.2))

	def gpsMeasurement(self,gpsInput):
		sigma = np.zeros((self.stateDim,self.stateDim))
		sigma[0,0] = 1.0/5**2
		sigma[1,1] = 1.0/5**2
		sigma[2,2] = 1.0/(5*np.pi/180)**2
		mu = np.array((gpsInput[0][0],gpsInput[0][1],gpsInput[1]))
		scores = np.zeros(self.numP)
		for i in range(self.numP):
			difference = self.particles[:,i]-mu
			scores[i] = np.exp(-0.5*np.dot( difference, np.dot(sigma,difference)))
		super(ParticleFilterDriving,self).resample(scores)

	def getMapPts(self,state=None,ptDist=np.arange(10,81,10)):
		if state is None:
			state = super(ParticleFilterDriving,self).state()
		idxTree = self.searchTree.query(state[0:2])
		idx = idxTree[1]
		#endIdx = np.argmax(self.S>(self.S[idx]+100))
		#if endIdx==0:
		#	endIdx = self.map.shape[0]-1
		endIdx = idx+1000
		mapPts = self.mapInterp[idx:endIdx,:]

		if not ptDist.any():
			selectPts = mapPts.copy()
		else:
			#S = np.insert(np.cumsum(np.sqrt( np.diff(mapPts[:,0])**2 + np.diff(mapPts[:,1])**2 + np.diff(mapPts[:,2])**2)),0,0)
			selectPts = np.ones((len(ptDist),4))
			#s = self.S[idx:endIdx]-self.S[idx]
			#selectPts[:,0] = np.interp(ptDist,s,mapPts[:,0])
			#selectPts[:,1] = np.interp(ptDist,s,mapPts[:,1])
			#selectPts[:,2] = np.interp(ptDist,s,mapPts[:,2])
			selectPts = mapPts[idx+ptDist*10,:].copy()

		#mapPos = np.linalg.solve(self.trMap[idx,:,:], selectPts.transpose())
		selectPts[:,0] = selectPts[:,0]-state[0]
		#selectPts[:,1] = mapPos[1,:]
		selectPts[:,1] = -selectPts[:,1]+mapPts[0,1]+self.cam['t_y']
		selectPts[:,2] = selectPts[:,2]-state[1]
		'''
		mapPts[:,0] = mapPts[:,0] - state[0]
		mapPts[:,2] = mapPts[:,2] - state[1]
		selectPts = mapPts
		'''
		#print selectPts
		theta = -(np.pi/2-state[2])
		#theta = -state[2]
		rot = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
		selectPts[:,0:3] = np.dot(rot,selectPts[:,0:3].transpose()).transpose()
		print selectPts
		selectPix = np.around(np.dot(self.cam['KK'], np.divide(selectPts[:,0:3].transpose(),selectPts[:,2].transpose())))
		return selectPix[0:2,:]

	def visionMeasurement(self,visionInput):
		scores = np.zeros(self.numP)
		for i in range(self.numP):
			scores[i] = 1/np.linalg.norm(visionInput-self.getMapPts(self.particles[:,i]))
		super(ParticleFilterDriving,self).resample(scores)	
