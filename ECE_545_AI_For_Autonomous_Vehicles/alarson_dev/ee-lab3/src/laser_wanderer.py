#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point

SCAN_TOPIC = '/car/scan' # The topic to subscribe to for laser scans
CMD_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation' # The topic to publish controls to
POSE_TOPIC = '/car/car_pose' # The topic to subscribe to for current pose of the car
                                  # NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
VIZ_TOPIC = '/laser_wanderer/rollouts' # The topic to publish to for vizualizing
                                       # the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 1000 # The penalty to apply when a configuration in a rollout
                   # goes beyond the corresponding laser scan
                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''
class LaserWanderer:

  '''
  Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the 
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
  '''
  def __init__(self, phys, rollouts, deltas, speed, compute_time, laser_offset, car_length):
    # Store the params for later
    self.phys = phys
    self.car_length = car_length
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size=100)
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb)
    self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray, queue_size=10)
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub_cb)
    # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    
  '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''  
  def viz_sub_cb(self, msg):
    # Create the PoseArray to publish. Will contain N poses, where the n-th pose
    # represents the last pose in the n-th trajectory
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()
    pa.poses = []

    # Transform the last pose of each trajectory to be w.r.t the world and insert into
    # the pose array
    # Car Pose:
    x_car=msg.pose.position.x
    y_car=msg.pose.position.y
    theta_car=utils.quaternion_to_angle(msg.pose.orientation)
    rospy.loginfo("car_pose: %s, %s, %s"%(x_car,y_car,theta_car))
    
    # for each rollout, get the last pose vector
    # transform to car coord.  (translate then rotate should be fine?)
    for x, y, theta in self.rollouts[:,-1,]:
      
      rot = np.array([[np.cos(theta_car), -np.sin(theta_car)],[np.sin(theta_car), np.cos(theta_car)]])
      r = np.matmul(rot, np.array([x,y]))
      theta += theta_car
      point = Point(x=r[0]+x_car, y=r[1]+y_car)
      orientation = utils.angle_to_quaternion(theta)
      pa.poses.append(Pose(position=point, orientation=orientation))


    self.viz_pub.publish(pa)
    
    
  '''
  Compute the cost of one step in the trajectory. It should penalize the magnitude
  of the steering angle. It should also heavily penalize crashing into an object
  (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
  '''  
  def compute_cost(self, delta, pose, laser_msg):
  
    # Initialize the cost to be the magnitude of delta
    # Consider the line that goes from the robot to the rollout pose
    # Compute the angle of this line with respect to the robot's x axis
    # Find the laser ray that corresponds to this angle
    # Add MAX_PENALTY to the cost if the distance from the robot to the rollout_pose 
    # is greater than the laser ray measurement - np.abs(self.laser_offset)
    # Return the resulting cost
    # Things to think about:
    #   What if the angle of the pose is less (or greater) than the angle of the
    #   minimum (or maximum) laser scan angle
    #   What if the corresponding laser measurement is NAN or 0?
    # NOTE THAT NO COORDINATE TRANSFORMS ARE NECESSARY INSIDE OF THIS FUNCTION
    cost = np.abs(delta)

    # for physical simulation - use the chassis size to compute car extent vectors
    if self.phys:
      # car extent vectors c_l and c_r, rotated by theta_pose, and translated to pose location:
      r = np.array([[np.cos(pose[2]), -np.sin(pose[2])],[np.sin(pose[2]), np.cos(pose[2])]])
      cl = np.matmul(r,np.array([self.car_length, self.car_length/2]))
      cr = np.matmul(r,np.array([self.car_length, -self.car_length/2]))
      vl = np.array([pose[0] + cl[0], pose[1] + cl[1]])
      vr = np.array([pose[0] + cl[0], pose[1] + cr[1]])
      dl = np.sqrt(vl.dot(vl))
      dr = np.sqrt(vr.dot(vr))
      thetal = np.arctan(vl[1]/vl[0])
      thetar = np.arctan(vr[1]/vr[0])
      idxl = ((thetal + np.pi)/2/np.pi)*720
      ll = np.array([laser_msg.ranges[int(np.floor(idxl))], laser_msg.ranges[int(np.ceil(idxl))]])
      ll = np.array([10.0 if x is np.NAN else x for x in ll])
      ll_ave = ll.sum()/len(ll)
      idxr = ((thetar + np.pi)/2/np.pi)*720
      lr = np.array([laser_msg.ranges[int(np.floor(idxr))], laser_msg.ranges[int(np.ceil(idxr))]])
      lr = np.array([10.0 if x is np.NAN else x for x in lr])
      lr_ave = lr.sum()/len(lr)
      if ll_ave < dl:
        cost+=MAX_PENALTY
      if lr_ave < dr:
        cost+=MAX_PENALTY
      
    else:
      # may want to check that rollout_pose x is not zero
      theta=np.arctan(pose[1]/pose[0])
      # map theta to index
      idx = ((theta + np.pi)/2/np.pi)*720
      #rospy.loginfo("pose: x is %f y is %f"%(pose[0],pose[1]))
      #rospy.loginfo("theta is %f; idx is %f"%(theta,idx))
      # bounds check
      if np.floor(idx) < 0:
        rospy.loginfo("Laser index less than 0")
      elif np.ceil(idx) > len(laser_msg.ranges)-1:
        rospy.loginfo("Laser greater than len(ranges) ")
      #else:
      #  rospy.loginfo("ranges found: %s and %s"%(laser_msg.ranges[int(np.floor(idx))], laser_msg.ranges[int(np.ceil(idx))]))
      # pose has x, y, delta:  at laser scan delta, add 
      
      # at theta we get laser d.  check versus pose x and y
      d = np.array([laser_msg.ranges[int(np.floor(idx))], laser_msg.ranges[int(np.ceil(idx))]])
      #d = np.nan_to_num(d, nan=10.0)
      d = np.array([10.0 if x is np.NAN else x for x in d])
      
      d_ave = d.sum()/len(d)
      p_d = np.sqrt(pose[0]**2 + pose[1]**2)
      metric = d_ave - self.laser_offset - p_d
      if metric < 0:
        cost += MAX_PENALTY
      elif metric < 0.2:  # add a small penalty for closeness..
        cost += 0.5
      #rospy.loginfo("Cost for theta %f is %f"%(theta, cost))
    return cost
    
  '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''
  def wander_cb(self, msg):
    
    # A N dimensional matrix that should be populated with the costs of each
    # trajectory up to time t <= T
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) 
    depth = 0
      

    # Evaluate the cost of each trajectory. Each iteration of the loop should calculate
    # the cost of each trajectory at time t = traj_depth and add those costs to delta_costs
    # as appropriate
    start = rospy.Time.now().to_sec() # Get the time at which this function started
    cur_time=start
    while depth < self.rollouts.shape[1]:
      # ror each rollout, note index of rollouts and deltas are same
      for i in range(self.rollouts.shape[0]):
        cost = self.compute_cost(self.deltas[i], self.rollouts[i,depth,:],msg)
        delta_costs[i] += cost
      cur_time== rospy.Time.now().to_sec()
      if cur_time-start > self.compute_cost:
        break
      else: 
        depth += 1

    rospy.loginfo("Computed %d depth of trajectories in %f time"%(depth, cur_time-start))
    for i in range(self.deltas.shape[0]):
      rospy.loginfo("For trajectory %f, cost is %f"%(self.deltas[i], delta_costs[i]))

    idx_min = np.argmin(delta_costs)
    rospy.loginfo("Trajectory is %f"%self.deltas[idx_min])
      

     # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = self.deltas[idx_min]
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)
    
'''
Apply the kinematic model to the passed pose and control
  pose:  current car pose [x, y, theta]
  control: controls to be applied [v, delta, dt]
  car_length: length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(pose, control, car_length):
  # Apply the kinematic model
  # Given a step, update kinematics (how does carlength play in?) 
  # delta and theta (heading) are not the same
  # x=> x + vx*dt
  # y=> y + vy*dt
  # theta => theta + dtheta

  x = pose[0] + np.cos(pose[2])*control[0]*control[2]
  y = pose[1] + np.sin(pose[2])*control[0]*control[2]
  dtheta = (control[0]/car_length) * np.tan(control[1]) * control[2]
  theta = pose[2] + dtheta
  return [x,y,theta]
    
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):
  
  rollout = np.zeros((controls.shape[0],3), dtype=np.float)
  for i in range(controls.shape[0]):
    if i==0:
      rollout[i,:] = kinematic_model_step(init_pose, controls[i,:], car_length)
    else:
      rollout[i,:] = kinematic_model_step(rollout[i-1,:], controls[i,:], car_length)
  return rollout
   
'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  deltas = np.arange(min_delta, max_delta, delta_incr)
  N = deltas.shape[0]
  rospy.loginfo("Generated %d deltas"%N)
  
  init_pose = np.array([0,0,0], dtype=np.float)
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in xrange(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed  # In this case, controls are constant for the rollouts..  could potentially be different?
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    rospy.loginfo("Controls shape: (%s, %s)"%controls.shape)
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)
    
  return rollouts, deltas

    

def main():

  rospy.init_node('laser_wanderer', anonymous=True)

  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LaserWanderer class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system  
  # YOUR CODE HERE
  speed = rospy.get_param("~speed")  #1.0
  min_delta = rospy.get_param("~min_delta") # -0.34
  max_delta =  rospy.get_param("~max_delta") #-0.34
  phys = rospy.get_param("~phys")  # toggle physical robot
  N = rospy.get_param("~N") #0.341
  delta_incr = max_delta/N
  rospy.loginfo("Num trajs: %d, inc: %f"%(N,delta_incr))
  dt = rospy.get_param("~dt") # 0.01 # Default val: 0.01
  T = rospy.get_param("~T") # 200 # Starting val: 300
  compute_time = rospy.get_param("~compute_time") # 0.09 # Default val: 0.09
  laser_offset = rospy.get_param("~laser_offset") # 0.9 # Starting val: 1.0
  
  # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
  car_length = rospy.get_param("/car/vesc/chassis_length", 0.33) 
  
  # Generate the rollouts
  rollouts, deltas = generate_mpc_rollouts( speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
  rospy.loginfo("rollouts.shape: (%s, %s, %s) "%rollouts.shape)

  # Create the LaserWanderer                                         
  lw = LaserWanderer(phys, rollouts, deltas, speed, compute_time, laser_offset, car_length)
  
  # Keep the node alive
  rospy.spin()
  

if __name__ == '__main__':
  main()
