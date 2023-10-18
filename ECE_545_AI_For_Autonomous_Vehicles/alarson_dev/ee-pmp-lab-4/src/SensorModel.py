#!/usr/bin/env python

import numpy as np
import rospy
import range_libc
import time
from threading import Lock
from nav_msgs.srv import GetMap
import rosbag
import matplotlib.pyplot as plt
import utils as Utils
from sensor_msgs.msg import LaserScan

THETA_DISCRETIZATION = 112 # Discretization of scanning angle
INV_SQUASH_FACTOR = 0.2   # Factor for helping the weight distribution to be less peaked
# What happens when this is varied?  basically bring up more data into the image visualization

# YOUR CODE HERE (Set these values and use them in precompute_sensor_model)
Z_SHORT = 0.02 # Weight for short reading
Z_MAX =  0.02 # Weight for max reading
Z_RAND =  0.05 # Weight for random reading
SIGMA_HIT = 4.0 # Noise value for hit reading (in pixels)
Z_HIT =  0.91 # Weight for hit reading
LAMBDA = 0.1 # 

''' 
  Weights particles according to their agreement with the observed data
'''
class SensorModel:
	
  '''
  Initializes the sensor model
    scan_topic: The topic containing laser scans
    laser_ray_step: Step for downsampling laser scans
    exclude_max_range_rays: Whether to exclude rays that are beyond the max range
    max_range_meters: The max range of the laser
    map_msg: A nav_msgs/MapMetaData msg containing the map to use
    
    particles: The particles to be weighted - these are all the hypotheses that exist in the real world
      for poses - note the angle step of 25 => +/- 12 deg
    weights: The weights of the particles
    state_lock: Used to control access to particles and weights
  '''
  def __init__(self, scan_topic, laser_ray_step, exclude_max_range_rays, 
               max_range_meters, map_msg, particles, weights, state_lock=None):
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
    self.particles = particles
    self.weights = weights
    
    self.LASER_RAY_STEP = laser_ray_step # Step for downsampling laser scans
    self.EXCLUDE_MAX_RANGE_RAYS = exclude_max_range_rays # Whether to exclude rays that are beyond the max range
    self.MAX_RANGE_METERS = max_range_meters # The max range of the laser
    
    oMap = range_libc.PyOMap(map_msg) # A version of the map that range_libc can understand
    max_range_px = int(self.MAX_RANGE_METERS / map_msg.info.resolution) # The max range in pixels of the laser
    self.range_method = range_libc.PyCDDTCast(oMap, max_range_px, THETA_DISCRETIZATION) # The range method that will be used for ray casting
    #self.range_method = range_libc.PyRayMarchingGPU(oMap, max_range_px) # The range method that will be used for ray casting
   
    self.range_method.set_sensor_model(self.precompute_sensor_model(max_range_px)) # Load the sensor model expressed as a table
    
    self.queries = None # Do not modify this variable
    self.ranges = None # Do not modify this variable
    self.laser_angles = None # The angles of each ray
    self.downsampled_angles = None # The angles of the downsampled rays 
    self.do_resample = False # Set so that outside code can know that it's time to resample
    
    # Subscribe to laser scans
    self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_cb, queue_size=1)    

  '''
    Downsamples laser measurements and applies sensor model
      msg: A sensor_msgs/LaserScan
  '''    
  def lidar_cb(self, msg):
    self.state_lock.acquire()
 
    # Compute the observation obs
    #   obs is a a two element tuple
    #   obs[0] is the downsampled ranges
    #   obs[1] is the downsampled angles
    #   Note it should be the case that obs[0].shape[0] == obs[1].shape[0]
    #   Each element of obs must be a numpy array of type np.float32
    #   Use self.LASER_RAY_STEP as the downsampling step
    #   Keep efficiency in mind, including by caching certain things that won't change across future iterations of this callback
    #   and vectorizing computations as much as possible
    #   Set all range measurements that are NAN or 0.0 to self.MAX_RANGE_METERS
    #   You may choose to use self.laser_angles and self.downsampled_angles here
    # YOUR CODE HERE

    # starting with mod 3 => 720 samples downsampled to 240
    
    rospy.loginfo("Computing lidar scan, size of data %s, angle_inc: %s, angle_min: %s, angle_max: %s range_min: %s, range_max: %s"
      %(len(msg.ranges), msg.angle_increment, msg.angle_min, msg.angle_max, msg.range_min, msg.range_max))
    if not self.downsampled_angles:  # cache downsampled angles once - assumes it is constant
      self.downsampled_angles=[]
      for i in xrange(len(msg.ranges)):
        if i % self.LASER_RAY_STEP == 0:
          self.downsampled_angles.append(msg.angle_min+i*msg.angle_increment)
          #rospy.loginfo("Adding downsampled angle %s"%(msg.angle_min+i*msg.angle_increment))
    ranges=[]
    for i in xrange(len(msg.ranges)):
      if i % self.LASER_RAY_STEP == 0:
        val = msg.ranges[i]
        #rospy.loginfo("val type: %s value:  %s, isNAN: %s"%(type(val), val, np.isnan(val)))
        if np.isnan(val) or val == 0.0:
          #rospy.loginfo("Scrubbing data: %s"%val)
          ranges.append(self.MAX_RANGE_METERS)
        else:
          ranges.append(np.float32(val))

    obs = np.zeros((2,len(ranges)), dtype=np.float32)
    obs[0,:] = np.array(self.downsampled_angles, dtype=np.float32)
    obs[1,:] = np.array(ranges, dtype=np.float32)
    #rospy.loginfo("Obs created, obs array shape %s, %s"%obs.shape)
    #rospy.loginfo("weights sum: %s"%self.weights.sum())
    self.apply_sensor_model(self.particles, obs, self.weights)
    #rospy.loginfo("Model applied.  weights sum: %s"%weights.sum())
    self.weights /= np.sum(self.weights)
    
    self.last_laser = msg
    self.do_resample = True
    self.state_lock.release()
    
  '''
    Compute table enumerating the probability of observing a measurement 
    given the expected measurement
    Element (r,d) of the table is the probability of observing measurement r (in pixels)
    when the expected measurement is d (in pixels)
    max_range_px: The maximum range in pixels
    Returns the table (which is a numpy array with dimensions [max_range_px+1, max_range_px+1]) 
  '''  
  def precompute_sensor_model(self, max_range_px):
    # Sensor model is a range model - probability that range is r, given actual range d

    table_width = int(max_range_px) + 1
    rospy.loginfo("Computing sensor model table [Size is %s x %s in pixels]"%(table_width, table_width))
    sensor_model_table = np.zeros((table_width,table_width))

    # Populate sensor_model_table according to the laser beam model specified
    # in CH 6.3 of Probabilistic Robotics
    # Note: no need to use any functions from utils.py to compute between world
    #       and map coordinates here    
    # YOUR CODE HERE
    # Pseudo-code
    # for d in xrange(table_width):
    #   possibly some stuff here
    #   for r in xrange(table_width):
    #     Populate the sensor model table at (r,d) with the probability of 
    #     observing measurement r (in pixels)
    #     when the expected measurement is d (in pixels)
    # Note that the 'self' parameter is completely unused in this function

    N = sensor_model_table.shape[0]
    for d in range(N):
      eta = 1.0/(1.0 - np.exp(-LAMBDA*d)) if d>0 else 0.0  # not sure what to do when  d = 0, disregard this I guess
      for r in range(N):
        phit = ((2*np.pi*SIGMA_HIT**2)**-0.5 ) * np.exp(-0.5*(d - r)**2/SIGMA_HIT**2)
        psh = eta*LAMBDA*np.exp(-LAMBDA*r) if r <= d else 0.0
        pran = 1.0/(N)
        pmax = 1.0 if r==int(max_range_px) else 0.0
        p = Z_HIT*phit + Z_MAX*pmax + Z_RAND * pran + Z_SHORT*psh
        sensor_model_table[r,d] = p
        #if np.random.randint(100)==17:
        #  rospy.loginfo("Table, r: %s, d: %s, p: %s"%(r, d, p))

    #for i in xrange(10):
    #  rospy.loginfo("Table, r: %s, d: 2, p: %s"%(i, sensor_model_table[i,2]))
    #  rospy.loginfo("Table, r: %s, d: 10, p: %s"%(i, sensor_model_table[i,10]))
    
    rospy.loginfo("Sensor Model: Table computed. sum(p[:,15]): %s"%np.sum(sensor_model_table[:,15]))
    
        
    # sensor_model_table is passed to the range_libc model which computes in pixel space
    return sensor_model_table

  '''
    Updates the particle weights in-place based on the observed laser scan
      proposal_dist: The particles
      obs: The most recent observation
      weights: The weights of each particle
  '''
  def apply_sensor_model(self, proposal_dist, obs, weights):
        
    obs_ranges = obs[1]
    obs_angles = obs[0]
    num_rays = obs_angles.shape[0]
    
    
    # Debug code
    #rospy.loginfo("Weights: size: %s, sum: %s"%(weights.shape[0], np.sum(weights)))
    #for idx in range(obs_ranges.shape[0]):
    #  rospy.loginfo("Obs r: %s, theta: %s"%(obs_ranges[idx], obs_angles[idx]))

    # Only allocate buffers once to avoid slowness
    if not isinstance(self.queries, np.ndarray):
      self.queries = np.zeros((proposal_dist.shape[0],3), dtype=np.float32)
      self.ranges = np.zeros(num_rays*proposal_dist.shape[0], dtype=np.float32)
    
    self.queries[:,:] = proposal_dist[:,:]

    # Raycasting to get expected measurements
    self.range_method.calc_range_repeat_angles(self.queries, obs_angles, self.ranges)

    # Evaluate the sensor model
    self.range_method.eval_sensor_model(obs_ranges, self.ranges, weights, num_rays, proposal_dist.shape[0])

    # Squash weights (inplace) to prevent too much peakiness
    np.power(weights, INV_SQUASH_FACTOR, weights)

    #rospy.loginfo("Finished applying sensor model, weights type: %s, len: %s, max: %s, min: %s, mean: %s"%(
    #  type(weights),weights.shape[0], weights.max(), weights.min(), weights.sum()/weights.shape[0]))
    #rospy.loginfo("Weights: %s"%",".join([str(x) for x in weights[0:10]]))

'''
  Code for testing SensorModel
'''

MAP_TOPIC = 'static_map'

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  bag_path = rospy.get_param("~bag_path", '/home/car-user/racecar_ws/src/ta_lab2/bags/laser_scans/laser_scan1.bag')
  scan_topic = rospy.get_param("~scan_topic", "/scan") # The topic containing laser scans
  laser_ray_step = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
  exclude_max_range_rays = bool(rospy.get_param("~exclude_max_range_rays")) # Whether to exclude rays that are beyond the max range
  max_range_meters = float(rospy.get_param("~max_range_meters")) # The max range of the laser               

  print 'Bag path: ' + bag_path

  # Use the 'static_map' service (launched by MapServer.launch) to get the map
  print("Getting map from service: ", MAP_TOPIC)
  rospy.wait_for_service(MAP_TOPIC)
  map_msg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map # The map, will get passed to init of sensor model
  map_info = map_msg.info # Save info about map for later use    

  print 'Creating permissible region'
  # Create numpy array representing map for later use
  array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
  permissible_region = np.zeros_like(array_255, dtype=bool)
  permissible_region[array_255==0] = 1 # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
                                            # With values 0: not permissible, 1: permissible
  permissible_y, permissible_x = np.where(permissible_region == 1)
  
  rospy.loginfo("array_255.shape: %s, %s"%array_255.shape)
  rospy.loginfo("Permissible x type: %s"%type(permissible_x[0]))
  rospy.loginfo("Permissible size x: %s; y: %s"%(len(permissible_x), len(permissible_y)))
  rospy.loginfo("Map scale (pixel to meter): %s"%map_info.resolution)
  # visualize map
  # Potentially downsample permissible_x and permissible_y here
  
  dsp_x=[]; dsp_y=[]
  # Downsampling?
  ds=4
  for i in range(permissible_x.shape[0]):
    if np.random.randint(ds) == 0:
      dsp_x.append( permissible_x[i])
      dsp_y.append( permissible_y[i])
  dsp_x = np.array(dsp_x, dtype=np.int32)
  dsp_y = np.array(dsp_y, dtype=np.int32)
  rospy.loginfo("Downsampling permissible size x: %s; y: %s"%(dsp_x.shape[0], dsp_y.shape[0]))
  angle_step = 20
  particles = np.zeros((angle_step * dsp_x.shape[0],3))
  for i in xrange(angle_step):
    particles[i*(particles.shape[0]/angle_step):(i+1)*(particles.shape[0]/angle_step),0] = dsp_x[:]
    particles[i*(particles.shape[0]/angle_step):(i+1)*(particles.shape[0]/angle_step),1] = dsp_y[:]
    particles[i*(particles.shape[0]/angle_step):(i+1)*(particles.shape[0]/angle_step),2] = i*(2*np.pi / angle_step)
  #rospy.loginfo( 'Creating particles angle_step * N permissible coords hypotheses: %s'%angle_step*permissible_x.shape[0] )
  
  # Note - this takes particles and transforms them into world space, in place.
  Utils.map_to_world(particles, map_info)
  weights = np.ones(particles.shape[0]) / float(particles.shape[0])
  
  print 'Initializing sensor model'
  sm = SensorModel(scan_topic, laser_ray_step, exclude_max_range_rays, 
                   max_range_meters, map_msg, particles, weights)
  
  # Give time to get setup
  rospy.sleep(1.0)
  #exit()
  # Load laser scan from bag
  bag = rosbag.Bag(bag_path)
  for _, msg, _ in bag.read_messages(topics=['/scan']):
    laser_msg = msg
    break

  w_min = np.amin(weights)
  w_max = np.amax(weights)
  
  
  # each of the provided bag has 1 message - so this just grabs the laser scan data
  pub_laser = rospy.Publisher(scan_topic, LaserScan, queue_size = 1) # Publishes the most recent laser scan
  print("Starting analysis, this could take awhile...")
  while not isinstance(sm.queries, np.ndarray):
    pub_laser.publish(laser_msg)
    rospy.sleep(1.0)
 
  rospy.sleep(1.0) # Make sure there's enough time for laserscan to get lock
  
  print 'Going to wait for sensor model to finish'
  sm.state_lock.acquire()
  print 'Done, preparing to plot'
  weights = weights.reshape((angle_step, -1))  # new shape is inferred from dimensions.. 
  
  weights = np.amax(weights, axis=0)  # grabs the highest value along the column axis (axis 0) 
  print map_msg.info.height
  print map_msg.info.width
  print weights.shape
  w_min = np.amin(weights)
  w_max = np.amax(weights)
  print 'w_min = %f'%w_min
  print 'w_max = %f'%w_max
  weights = 0.9*(weights-w_min)/(w_max-w_min) + 0.1
  rospy.loginfo("renormalized weights: max: %s, min: %s, ave: %s"%(weights.max(), weights.min(), weights.sum()/weights.shape[0]))
  rospy.loginfo("highest weight values:")

 
  rospy.loginfo("Type of weights: %s"%type(weights[0]))
  img = np.zeros((map_msg.info.height,map_msg.info.width))
  c=0 # display some of the highest weights
  for i in xrange(dsp_x.shape[0]):
    if weights[i] > 0.2 and c < 60:
      rospy.loginfo("High weight: %s, x: %s, y: %s"%(weights[i], dsp_x[i],dsp_y[i] ))
      c+=1
    img[dsp_y[i],dsp_x[i]] = weights[i]
  plt.imshow(img)
  plt.show()
  