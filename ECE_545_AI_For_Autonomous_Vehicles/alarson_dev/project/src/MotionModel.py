#!/usr/bin/env python

import rospy
import numpy as np
import Utils
from std_msgs.msg import Float64
from threading import Lock
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
import matplotlib.pyplot as plt

# YOUR CODE HERE (Set these values and use them in motion_cb)
KM_V_NOISE = 0.02 # Kinematic car velocity noise std dev
KM_DELTA_NOISE = 0.02# Kinematic car delta noise std dev
KM_X_FIX_NOISE = 0.02# Kinematic car x position constant noise std dev
KM_Y_FIX_NOISE = 0.02# Kinematic car y position constant noise std dev
KM_THETA_FIX_NOISE = 0.02# Kinematic car theta constant noise std dev

'''
  Propagates the particles forward based on the velocity and steering angle of the car
'''
class KinematicMotionModel:

  '''
    Initializes the kinematic motion model
      motor_state_topic: The topic containing motor state information
      servo_state_topic: The topic containing servo state information    
      speed_to_erpm_offset: Offset conversion param from rpm to speed
      speed_to_erpm_gain: Gain conversion param from rpm to speed
      steering_angle_to_servo_offset: Offset conversion param from servo position to steering angle
      steering_angle_to_servo_gain: Gain conversion param from servo position to steering angle 
      car_length: The length of the car
      particles: The particles to propagate forward
      state_lock: Controls access to particles    
  '''
  def __init__(self, motor_state_topic, servo_state_topic, speed_to_erpm_offset, 
               speed_to_erpm_gain, steering_to_servo_offset,
               steering_to_servo_gain, car_length, particles, state_lock=None):
    self.last_servo_cmd = None # The most recent servo command
    self.last_vesc_stamp = None # The time stamp from the previous vesc state msg
    self.particles = particles
    self.SPEED_TO_ERPM_OFFSET = speed_to_erpm_offset # Offset conversion param from rpm to speed
    self.SPEED_TO_ERPM_GAIN   = speed_to_erpm_gain # Gain conversion param from rpm to speed
    self.STEERING_TO_SERVO_OFFSET = steering_to_servo_offset # Offset conversion param from servo position to steering angle
    self.STEERING_TO_SERVO_GAIN = steering_to_servo_gain # Gain conversion param from servo position to steering angle
    self.CAR_LENGTH = car_length # The length of the car
    
    # This just ensures that two different threads are not changing the particles
    # array at the same time. You should not have to deal with this.
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
      
    # This subscriber just caches the most recent servo position command
    self.servo_pos_sub  = rospy.Subscriber(servo_state_topic, Float64,
                                       self.servo_cb, queue_size=1)
    # Subscribe to the state of the vesc
    self.motion_sub = rospy.Subscriber(motor_state_topic, VescStateStamped, self.motion_cb, queue_size=1)                                       

  '''
    Caches the most recent servo command
      msg: A std_msgs/Float64 message
  '''
  def servo_cb(self, msg):
    #rospy.loginfo("Servo cb: msg: %s"%msg.data)
    self.last_servo_cmd = msg.data # Update servo command

  '''
    Converts messages to controls and applies the kinematic car model to the
    particles
      msg: a vesc_msgs/VescStateStamped message
  '''
  def motion_cb(self, msg):
    self.state_lock.acquire()
    if self.last_servo_cmd is None:
      self.state_lock.release()
      return

    if self.last_vesc_stamp is None:
      #rospy.loginfo("In motion_cb, setting last_vesc_stamp to %s, speed is: %s"%(msg.header.stamp, msg.state.speed))
      #rospy.loginfo("stamp type: %s, to_sec(): %s"%(type(msg.header.stamp),msg.header.stamp.to_sec()))
      self.last_vesc_stamp = msg.header.stamp
      self.state_lock.release()
      return
    #rospy.loginfo("In motion_cb, last_vesc_stamp is %s, speed is: %s"%(msg.header.stamp, msg.state.speed))
    
    # Convert raw msgs to controls
    # Note that control_val = (raw_msg_val - offset_param) / gain_param
    # E.g: curr_speed = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    start = rospy.Time.now().to_sec()
    speed_mean = (msg.state.speed - self.SPEED_TO_ERPM_OFFSET) / self.SPEED_TO_ERPM_GAIN
    delta_mean = (self.last_servo_cmd - self.STEERING_TO_SERVO_OFFSET) / self.STEERING_TO_SERVO_GAIN
    dt = msg.header.stamp.to_sec() - self.last_vesc_stamp.to_sec()
    #rospy.loginfo("motion model cb: updating partices dt: %s, delta: %s, speed: %s"%(dt, delta_mean, speed_mean))
    self.last_vesc_stamp = msg.header.stamp
    # Now get values of raw params with noise added
    N = self.particles.shape[0]
    # vary the initial values - standard for particle filter models
    x_initial = np.random.normal(self.particles[:,0], KM_X_FIX_NOISE)  
    y_initial = np.random.normal(self.particles[:,1], KM_Y_FIX_NOISE)
    theta_initial = np.random.normal(self.particles[:,2], KM_THETA_FIX_NOISE)
    # vary the commands in the motion model
    delta_cmd = np.random.normal(delta_mean, KM_DELTA_NOISE, N)
    speed_cmd = np.random.normal(speed_mean, KM_V_NOISE, N)
    beta = np.arctan(0.5*np.tan(delta_cmd))
    theta = theta_initial + (speed_cmd/self.CAR_LENGTH)*np.sin(2*beta)*dt
    # step particles foward by dt
    y = y_initial + (self.CAR_LENGTH/ np.sin(2*beta)) * ( -np.cos(theta) + np.cos(theta_initial))
    x = x_initial + (self.CAR_LENGTH/ np.sin(2*beta)) * ( np.sin(theta) - np.sin(theta_initial))

    self.particles[:,0] = x
    self.particles[:,1] = y
    self.particles[:,2] = theta
    end = rospy.Time.now().to_sec()
    #rospy.loginfo("Finished computation with %d particles, took %s seconds"%(N, (end-start)))
    # Propagate particles forward in place
      # Sample control noise and add to nominal control
      # Make sure different control noise is sampled for each particle
      # Propagate particles through kinematic model with noisy controls
      # Sample model noise for each particle
      # Limit particle theta to be between -pi and pi
      # Vectorize your computations as much as possible
      # All updates to self.particles should be in-place
    # YOUR CODE HERE

    self.last_vesc_stamp = msg.header.stamp    
    self.state_lock.release()

'''
  Code for testing motion model
'''

TEST_SPEED = 2.0 # meters/sec
TEST_STEERING_ANGLE = -0.24 # radians
TEST_DT = 0.6 # seconds

if __name__ == '__main__':
  MAX_PARTICLES = 10000  ## Simply runs a simulation of the kinematics
  
  rospy.init_node("odometry_model", anonymous=True) # Initialize the node
  particles = np.zeros((MAX_PARTICLES,3)) # for motion model demo - particles start at 0,0,0

  # Load params
  motor_state_topic = rospy.get_param("~motor_state_topic", "/vesc/sensors/core") # The topic containing motor state information
  servo_state_topic = rospy.get_param("~servo_state_topic", "/vesc/sensors/servo_position_command") # The topic containing servo state information
  speed_to_erpm_offset = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0)) # Offset conversion param from rpm to speed
  speed_to_erpm_gain = float(rospy.get_param("/vesc/speed_to_erpm_gain", 4350))   # Gain conversion param from rpm to speed
  steering_angle_to_servo_offset = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5)) # Offset conversion param from servo position to steering angle
  steering_angle_to_servo_gain = float(rospy.get_param("/vesc/steering_angle_to_servo_gain", -1.2135)) # Gain conversion param from servo position to steering
  car_length = float(rospy.get_param("/car_kinematics/car_length", 0.33)) # The length of the car
    
  # Going to fake publish controls
  servo_pub = rospy.Publisher(servo_state_topic, Float64, queue_size=1)
  vesc_state_pub = rospy.Publisher(motor_state_topic, VescStateStamped, queue_size=1)
  
  kmm = KinematicMotionModel(motor_state_topic, servo_state_topic, speed_to_erpm_offset, 
                             speed_to_erpm_gain, steering_angle_to_servo_offset,
                             steering_angle_to_servo_gain, car_length,particles)
  
  # Give time to get setup
  rospy.sleep(1.0)
  
  # Send initial position and vesc state  
  servo_msg = Float64()
  servo_msg.data = steering_angle_to_servo_gain*TEST_STEERING_ANGLE+steering_angle_to_servo_offset
  
  servo_pub.publish(servo_msg)
  rospy.sleep(1.0)
  
  vesc_msg = VescStateStamped()
  vesc_msg.header.stamp = rospy.Time.now()
  vesc_msg.state.speed = speed_to_erpm_gain*TEST_SPEED+speed_to_erpm_offset  
  vesc_state_pub.publish(vesc_msg)
  
  rospy.sleep(TEST_DT)
  
  vesc_msg.header.stamp = rospy.Time.now()
  vesc_state_pub.publish(vesc_msg)
  
  rospy.sleep(1.0)
  
  kmm.state_lock.acquire()
  # Visualize particles
  plt.xlabel('x')
  plt.ylabel('y')
  plt.scatter([0],[0], c='r')
  plt.scatter(particles[:,0], particles[:,1], c='b')
  plt.show()
  kmm.state_lock.release()
