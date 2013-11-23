import numpy as np

class ParticleFilter(object):

	def __init__(self,initState,initVar,numParticles):
		self.numP = numParticles
		self.stateDim = len(initState)
		assert self.stateDim==len(initVar), "state dimension mismatch"
		assert self.numP > 0, "need at least 1 particle"
		self.particles = np.random.randn(self.stateDim,self.numP)
		for i in range(self.stateDim):
			self.particles[i,:] = self.particles[i,:]*initVar[i] + initState[i]

	def propogate(self,stateChange,stateChangeVar):
		assert self.stateDim== len(stateChange), "dimension mismatch"
		assert self.stateDim== len(stateChangeVar), "dimension mismatch"
		for i in range(self.stateDim):
			self.particles[i,:] = self.particles[i,:] + stateChange[i] \
				+ np.random.randn(1,self.numP)*stateChangeVar[i]

	def resample(self,scores):
		assert self.numP==len(scores), "need particle weight for every particle"
		m = np.mean(scores)
		r = np.random.rand()*m
		j = 0
		sum = scores[j]
		newParticles = np.zeros((self.stateDim,self.numP))
		for i in range(self.numP):
			while sum<r:
				j+=1
				sum += scores[j]
			newParticles[:,i] = self.particles[:,j]
			r+=m
		self.particles = newParticles.copy()

	def state(self):
		return np.mean(self.particles,1)
