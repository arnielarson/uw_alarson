#!/usr/bin/env python

import rospy 
import numpy as np
import scipy.special as special
import time
import utils as Utils
import tf.transformations
import tf
from threading import Lock

from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from ReSample import ReSampler
from SensorModel import SensorModel
from MotionModel import KinematicMotionModel

MAP_TOPIC = "static_map"
PUBLISH_PREFIX = '/pf/viz'
PUBLISH_TF = True

'''
  Implements particle filtering for estimating the state of the robot car
'''
class ParticleFilter():


  '''
  Initializes the particle filter
    n_particles: The number of particles
    n_viz_particles: The number of particles to visualize
    motor_state_topic: The topic containing motor state information
    servo_state_topic: The topic containing servo state information
    scan_topic: The topic containing laser scans
    laser_ray_step: Step for downsampling laser scans
    exclude_max_range_rays: Whether to exclude rays that are beyond the max range
    max_range_meters: The max range of the laser
    resample_type: Whether to use naiive or low variance sampling
    speed_to_erpm_offset: Offset conversion param from rpm to speed
    speed_to_erpm_gain: Gain conversion param from rpm to speed
    steering_angle_to_servo_offset: Offset conversion param from servo position to steering angle
    steering_angle_to_servo_gain: Gain conversion param from servo position to steering angle 
    car_length: The length of the car
  '''
  def __init__(self, n_particles, n_viz_particles,
               motor_state_topic, servo_state_topic, scan_topic, laser_ray_step,
               exclude_max_range_rays, max_range_meters, resample_type,
               speed_to_erpm_offset, speed_to_erpm_gain, steering_angle_to_servo_offset,
               steering_angle_to_servo_gain, car_length):
    self.N_PARTICLES = n_particles # The number of particles
                                   # In this implementation, the total number of 
                                   # particles is constant
    self.N_VIZ_PARTICLES = n_viz_particles # The number of particles to visualize

    self.particle_indices = np.arange(self.N_PARTICLES) # Cached list of particle indices
    self.particles = np.zeros((self.N_PARTICLES,3)) # Numpy matrix of dimension N_PARTICLES x 3
    self.weights = np.ones(self.N_PARTICLES) / float(self.N_PARTICLES) # Numpy matrix containig weight for each particle

    self.state_lock = Lock() # A lock used to prevent concurrency issues. You do not need to worry about this
    
    self.tfl = tf.TransformListener() # Transforms points between coordinate frames

    # Get the map
    print("Getting map from service: ", MAP_TOPIC)
    rospy.wait_for_service(MAP_TOPIC)
    map_msg = rospy.ServiceProxy(MAP_TOPIC, GetMap)().map # The map, will get passed to init of sensor model
    self.map_info = map_msg.info # Save info about map for later use    

    # Create numpy array representing map for later use
    self.array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
    self.permissible_region = np.zeros_like(self.array_255, dtype=bool)
    self.permissible_region[self.array_255==0] = 1 # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
                                              # With values 0: not permissible, 1: permissible    
    # From sensor model - get indices that are permissible (these are x,y pixels)
    self.permissible_y, self.permissible_x = np.where(self.permissible_region == 1)
    rospy.loginfo("Permissible points: %s, Max y value %s, Max x value: %s"%(self.permissible_x.shape[0], self.permissible_y.max(), self.permissible_x.max()))

    rospy.loginfo("Size of map: height: %s, width: %s"%(map_msg.info.height, map_msg.info.width))
    rospy.loginfo("shape[0]: %s, shape[1]: %s"%(self.array_255.shape[0], self.array_255.shape[1]))
    rospy.loginfo("Map spaces: %s, permissible spaces: %s"%(self.array_255.shape[0]*self.array_255.shape[1], self.permissible_region.sum()))
    
    
   
    # Publish particle filter state
    self.pub_tf = tf.TransformBroadcaster() # Used to create a tf between the map and the laser for visualization    
    self.pose_pub      = rospy.Publisher(PUBLISH_PREFIX + "/inferred_pose", PoseStamped, queue_size = 1) # Publishes the expected pose
    self.particle_pub  = rospy.Publisher(PUBLISH_PREFIX + "/particles", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    self.pub_laser     = rospy.Publisher(PUBLISH_PREFIX + "/scan", LaserScan, queue_size = 1) # Publishes the most recent laser scan
    self.pub_odom      = rospy.Publisher(PUBLISH_PREFIX + "/odom", Odometry, queue_size = 1) # Publishes the path of the car
    
    self.RESAMPLE_TYPE = resample_type # Whether to use naive or low variance sampling
    self.resampler = ReSampler(self.particles, self.weights, self.state_lock)  # An object used for resampling

    # An object used for applying sensor model
    self.sensor_model = SensorModel(scan_topic, laser_ray_step, exclude_max_range_rays, 
                                    max_range_meters, map_msg, self.particles, self.weights, 
                                    self.state_lock) 

    # An object used for applying kinematic motion model
    self.motion_model = KinematicMotionModel(motor_state_topic, servo_state_topic, 
                                             speed_to_erpm_offset, speed_to_erpm_gain, 
                                             steering_angle_to_servo_offset, steering_angle_to_servo_gain, 
                                             car_length, self.particles, self.state_lock)     
    
    # Subscribe to the '/initialpose' topic. Publised by RVIZ. See clicked_pose_cb function in this file for more info
    self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose_cb, queue_size=1)
    
    # Globally initialize the particles
    self.initialize_global()
    print('Initialization complete')

  '''
    Initialize the particles as uniform samples across the in-bounds regions of
    the map
  '''
  def initialize_global(self):
    self.state_lock.acquire()
    
    # Use self.permissible_region to get in-bounds states
    # Uniformally sample from in-bounds regions
    # Convert map samples (which are in pixels) to world samples (in meters/radians)
    #   Take a look at utils.py
    # Update particles in place
    # Update weights in place so that all particles have the same weight and the 
    # sum of the weights is one.
    # YOUR CODE HERE
    # Use the 'static_map' service (launched by MapServer.launch) to get the map
    
    
    # get N_PARTICLES random permissible locations 
    indices = np.random.randint(0,self.permissible_x.shape[0]-1,self.N_PARTICLES,dtype=np.int) 
    theta=np.array(2*np.random.random_sample(self.N_PARTICLES)*np.pi - np.pi) # initialize theta randomly
    
    for i in range(indices.shape[0]):
      #rospy.loginfo("Adding pose with x: %s; y: %s"%(permissible_x[indices[i]], permissible_y[indices[i]]))
      #rospy.loginfo("Value of raw map array here: %s"% self.array_255[permissible_y[indices[i]], permissible_x[indices[i]]])
      self.particles[i,0]=self.permissible_x[indices[i]]
      self.particles[i,1]=self.permissible_y[indices[i]]
    self.particles[:,2]=theta
    Utils.map_to_world(self.particles, self.map_info)
    # to world map
    #for i in range(self.particles.shape[0]):
    #  rospy.loginfo("Adding world pose with x: %s; y: %s, theta: %s"%(self.particles[i,0], self.particles[i,1], self.particles[i,2]))
      
    rospy.loginfo("Finished mappings for %s initial particles"%self.particles.shape[0])
    self.state_lock.release()
    self.visualize()
    
  '''
    Publish a tf between the laser and the map
    This is necessary in order to visualize the laser scan within the map
      pose: The pose of the laser w.r.t the map
      stamp: The time at which this pose was calculated, defaults to None - resulting
             in using the time at which this function was called as the stamp
  '''
  def publish_tf(self,pose,stamp=None):
    if stamp is None:
        stamp = rospy.Time.now()
    try:
      # Lookup the offset between laser and odom  
      delta_off, delta_rot = self.tfl.lookupTransform("/laser","/odom",rospy.Time(0))

      # Transform offset to be w.r.t the map
      off_x = delta_off[0]*np.cos(pose[2]) - delta_off[1]*np.sin(pose[2])
      off_y = delta_off[0]*np.sin(pose[2]) + delta_off[1]*np.cos(pose[2])

      # Broadcast the tf
      self.pub_tf.sendTransform((pose[0]+off_x,pose[1]+off_y,0.0),tf.transformations.quaternion_from_euler(0,0,pose[2]+tf.transformations.euler_from_quaternion(delta_rot)[2]),stamp,"/odom","/map")

    except (tf.LookupException): # Will occur if odom frame does not exist
      self.pub_tf.sendTransform((pose[0],pose[1],0),tf.transformations.quaternion_from_euler(0,0,pose[2]), stamp , "/laser", "/map")

  '''
    Returns a 3 element numpy array representing the expected pose given the 
    current particles and weights
    Uses weighted cosine and sine averaging to more accurately compute average theta
      https://en.wikipedia.org/wiki/Mean_of_circular_quantities
  '''
  def expected_pose(self):
    # YOUR CODE HERE
    rospy.loginfo("Particles: Min x: %s, Max x: %s, Min y: %s, Max y: %s"%(self.particles[:,0].min(), self.particles[:,0].max(), self.particles[:,1].min(), self.particles[:,1].max()))
    x=np.sum(self.particles[:,0]*self.weights)
    y=np.sum(self.particles[:,1]*self.weights)
    s=np.sum(np.sin(self.particles[:,2])*self.weights)
    c=np.sum(np.cos(self.particles[:,2])*self.weights)
    theta = np.arctan(s/c)
    if c <  0:
      theta += np.pi
    #elif s < 0:
    #  theta += 2*np.pi
    return np.array([x, y, theta])
    
  '''
    Callback for '/initialpose' topic. RVIZ publishes a message to this topic when you specify an initial pose 
    using the '2D Pose Estimate' button
    Reinitialize particles and weights according to the received initial pose
  '''
  def clicked_pose_cb(self, msg):
    self.state_lock.acquire()
    # Sample particles from a gaussian centered around the received pose
    # Updates the particles in place
    # Updates the weights to all be equal, and sum to one    
    # YOUR CODE HERE

    rospy.loginfo("Clicked Pose, updating particels")
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    
    # somewhat ugly..
    idx=0
    while idx<self.N_PARTICLES:
      i = np.random.randint(0,self.permissible_x.shape[0])
      x=self.permissible_x[i]
      y=self.permissible_y[i]
      theta=2*np.random.random_sample()*np.pi - np.pi
      t=np.array([[x,y,theta]])
      Utils.map_to_world(t, self.map_info)
      # get_prob
      d=np.sqrt(np.power(t[0,0]-px,2)+np.power(t[0,1]-py,2))
      # hack, just keep the closer ones, less than 15
      if d < 15:
        self.particles[idx,0] = t[0,0]
        self.particles[idx,1] = t[0,1]
        self.particles[idx,2] = t[0,2]
        idx+=1
     
    
    #Utils.map_to_world(self.particles, self.map_info)

    self.state_lock.release()
    self.visualize()
    
  '''
    Visualize the current state of the filter
   (1) Publishes a tf between the map and the laser. Necessary for visualizing the laser scan in the map
   (2) Publishes the most recent laser measurement. Note that the frame_id of this message should be '/laser'
   (3) Publishes a PoseStamped message indicating the expected pose of the car
   (4) Publishes a subsample of the particles (use self.N_VIZ_PARTICLES). 
       Sample so that particles with higher weights are more likely to be sampled.
  '''
  def visualize(self):
    #print 'Visualizing...'
    self.state_lock.acquire()
    self.inferred_pose = self.expected_pose()

    rospy.loginfo("Pose pub num conns: %s"%self.pose_pub.get_num_connections())  
    rospy.loginfo("inferred pose type: %s"%type(self.inferred_pose))
    if isinstance(self.inferred_pose, np.ndarray):
      if PUBLISH_TF:
        self.publish_tf(self.inferred_pose)
      ps = PoseStamped()
      ps.header = Utils.make_header("map")
      ps.pose.position.x = self.inferred_pose[0]
      ps.pose.position.y = self.inferred_pose[1]
      ps.pose.orientation = Utils.angle_to_quaternion(self.inferred_pose[2])  
      
      if(self.pose_pub.get_num_connections() > 0):
        rospy.loginfo("Visualizing particles [msg to /particles], inferred pose x: %s, y: %s, theta: %s"%(self.inferred_pose[0], self.inferred_pose[1], self.inferred_pose[2]))
        self.pose_pub.publish(ps)
      if(self.pub_odom.get_num_connections() > 0):
        odom = Odometry()
        odom.header = ps.header
        odom.pose.pose = ps.pose
        self.pub_odom.publish(odom)

    if self.particle_pub.get_num_connections() > 0:
      if self.particles.shape[0] > self.N_VIZ_PARTICLES:
        # randomly downsample particles
        proposal_indices = np.random.choice(self.particle_indices, self.N_VIZ_PARTICLES, p=self.weights)
        proposal_indices = np.random.choice(self.particle_indices, self.N_VIZ_PARTICLES)
        rospy.loginfo("Publishing particles [msg to /particles]")

        self.publish_particles(self.particles[proposal_indices,:])
      else:
        self.publish_particles(self.particles)
        
    if self.pub_laser.get_num_connections() > 0 and isinstance(self.sensor_model.last_laser, LaserScan):
      self.sensor_model.last_laser.header.frame_id = "/laser"
      self.sensor_model.last_laser.header.stamp = rospy.Time.now()
      self.pub_laser.publish(self.sensor_model.last_laser)
    self.state_lock.release()

  '''
  Helper function for publishing a pose array of particles
    particles: To particles to publish
  '''
  def publish_particles(self, particles):
    pa = PoseArray()
    pa.header = Utils.make_header("map")
    pa.poses = Utils.particles_to_poses(particles)
    self.particle_pub.publish(pa)

# Suggested main 
if __name__ == '__main__':
  rospy.init_node("particle_filter", anonymous=True) # Initialize the node
  
  n_particles = int(rospy.get_param("~n_particles")) # The number of particles
  n_viz_particles = int(rospy.get_param("~n_viz_particles")) # The number of particles to visualize
  motor_state_topic = rospy.get_param("~motor_state_topic", "/car/vesc/sensors/core") # The topic containing motor state information
  servo_state_topic = rospy.get_param("~servo_state_topic", "/car/vesc/sensors/servo_position_command") # The topic containing servo state information
  scan_topic = rospy.get_param("~scan_topic", "/car/scan") # The topic containing laser scans
  laser_ray_step = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
  exclude_max_range_rays = bool(rospy.get_param("~exclude_max_range_rays")) # Whether to exclude rays that are beyond the max range
  max_range_meters = float(rospy.get_param("~max_range_meters")) # The max range of the laser
  resample_type = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling

  speed_to_erpm_offset = float(rospy.get_param("/car/vesc/speed_to_erpm_offset", 0.0)) # Offset conversion param from rpm to speed
  speed_to_erpm_gain = float(rospy.get_param("/car/vesc/speed_to_erpm_gain", 4350))   # Gain conversion param from rpm to speed
  steering_angle_to_servo_offset = float(rospy.get_param("/car/vesc/steering_angle_to_servo_offset", 0.5)) # Offset conversion param from servo position to steering angle
  steering_angle_to_servo_gain = float(rospy.get_param("/car/vesc/steering_angle_to_servo_gain", -1.2135)) # Gain conversion param from servo position to steering angle    
  car_length = float(rospy.get_param("/car/vesc/chassis_length", 0.33)) # The length of the car
  
  # Create the particle filter  
  pf = ParticleFilter(n_particles, n_viz_particles,
                      motor_state_topic, servo_state_topic, scan_topic, laser_ray_step,
                      exclude_max_range_rays, max_range_meters, resample_type,
                      speed_to_erpm_offset, speed_to_erpm_gain, steering_angle_to_servo_offset,
                      steering_angle_to_servo_gain, car_length)
  
  while not rospy.is_shutdown(): # Keep going until we kill it
    # Callbacks are running in separate threads
    
    if pf.sensor_model.do_resample: # Check if the sensor model says it's time to resample
      pf.sensor_model.do_resample = False # Reset so that we don't keep resampling
      rospy.loginfo("Resampling")
      # Resample
      if pf.RESAMPLE_TYPE == "naiive":
        pf.resampler.resample_naiive()
      elif pf.RESAMPLE_TYPE == "low_variance":
        pf.resampler.resample_low_variance()
      else:
        print "Unrecognized resampling method: "+ pf.RESAMPLE_TYPE      
      
      pf.visualize() # Perform visualization


