import numpy as np
from ParticleFilter import *
from scipy import spatial

class ParticleFilterDriving(ParticleFilter):

	def __init__(self,initState,initVar,numParticles,map):
		ParticleFilter.__init__(self,initState,initVar,numParticles)
		self.map = map
		self.searchTree = spatial.KDTree(self.map[:,[0,2]])
	
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

	def getMapPts(self,state=None):
		if state is None:
			state = super(ParticleFilterDriving,self).state()
		idxTree = self.searchTree.query(state[0:2])
		idx = idxTree[1]
		endIdx = min(self.map.shape[0],idx+125)-1
		mapPts = self.map[idx:endIdx,:].copy()
		mapPts[:,0] = mapPts[:,0] - state[0]
		mapPts[:,2] = mapPts[:,2] - state[1]
		theta = -(np.pi/2-state[2])
		rot = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
		mapPts[:,0:3] = np.dot(rot,mapPts[:,0:3].transpose()).transpose()
		return mapPts.transpose()
