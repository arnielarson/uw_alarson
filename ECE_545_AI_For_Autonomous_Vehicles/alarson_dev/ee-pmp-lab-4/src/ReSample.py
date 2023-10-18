#!/usr/bin/env python

import rospy
import numpy as np
from threading import Lock

'''
  Provides methods for re-sampling from a distribution represented by weighted samples
'''
class ReSampler:

  '''
    Initializes the resampler
    particles: The particles to sample from
    weights: The weights of each particle
    state_lock: Controls access to particles and weights
  '''
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles 
    self.weights = weights
    
    # For speed purposes, you may wish to add additional member variable(s) that 
    # cache computations that will be reused in the re-sampling functions
    # YOUR CODE HERE?
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  '''
    Performs independently, identically distributed in-place sampling of particles
    assumes uniform distribution
  '''
  def resample_naive(self):
    self.state_lock.acquire()   
    
    self.particles[:,0] = np.random.choice(self.particles[:,0], size=self.particles.shape[0], p=self.weights) 
    

    self.state_lock.release()
  
  '''
    Performs in-place, lower variance sampling of particles
    (As discussed on pg 110 of Probabilistic Robotics)
  '''
  def resample_low_variance(self):
    self.state_lock.acquire()
    M = self.particles.shape[0]
    new_particles = np.zeros(self.particles.shape)
    r = np.random.uniform(0.0, 1.0/M)
    c = self.weights[0]
    i = 0; pi=0
    for m in range(M):
      U = r + m * 1.0/M
      while  U > c:
        i = i+1
        c += self.weights[i]
      new_particles[pi] = self.particles[i]
      pi+=1
    # copy in place
    for i in range(M):
      self.particles[i]=new_particles[i]    
    self.state_lock.release()
    
import matplotlib.pyplot as plt

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  n_particles = int(rospy.get_param("~n_particles",100)) # The number of particles    
  k_val = int(rospy.get_param("~k_val", 50)) # Number of particles that have non-zero weight
  resample_type = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling
  trials = int(rospy.get_param("~trials", 10)) # The number of re-samplings to do
  
  histogram = np.zeros(n_particles, dtype=np.float) # Keeps track of how many times
                                                    # each particle has been sampled
                                                    # across trials
  
  # np.newaxis takes the 1d array and makes it a 2d array, 5 columns, each with 1 element
  for i in xrange(trials):
    #rospy.loginfo("Starting trial: %s"%i)
    particles = np.repeat(np.arange(n_particles)[:,np.newaxis],3, axis=1) # Create a set of particles
                                                                          # Here their value encodes their index
    # Have increasing weights up until index k_val
    weights = np.arange(n_particles, dtype=np.float)
    weights[k_val:] = 0.0
    weights/=weights.sum()
    #weights[:] = weights[:] / np.sum(weights)  # note, this is already set up like a probability density.  
    
    rs = ReSampler(particles, weights) # Create the Resampler
  
    # Resample
    if resample_type == "naive":
      rs.resample_naive()
    elif resample_type == "low_variance":
      rs.resample_low_variance()
    else:
      print "Unrecognized resampling method: "+ resample_type     

    # Add the number times each particle was sampled    
    for j in xrange(particles.shape[0]):
      #rospy.loginfo("Updating histogram: adding particle: %s"%particles[j,0])
      #histogram[particles[j,0]] = histogram[particles[j,0]] + 1
      histogram[particles[j,0]]+=1

  # Display as histogram
  plt.bar(np.arange(n_particles), histogram)
  plt.xlabel('Particle Idx')
  plt.ylabel('# Of Times Sampled')
  plt.show()    

