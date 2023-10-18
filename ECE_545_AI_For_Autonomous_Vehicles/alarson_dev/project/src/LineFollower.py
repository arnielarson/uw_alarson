#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Point, Pose, PoseWithCovariance
from ackermann_msgs.msg import AckermannDriveStamped

import Utils

# The topic to publish control commands to
NAV_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
PLAN_TOPIC = '/planner_node/car_plan'
POSE_TOPIC = '/car/car_pose'
INITIALPOSE_TOPIC = '/initialpose'

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, waypoint_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed):
    # plan is a np array with shape (N,3)
    self.plan_idx = 0
    self.plan = plan
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights (why?)
    self.ww = waypoint_weight/(waypoint_weight+rotation_weight)
    self.rw = rotation_weight/(waypoint_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed
    self._i = 0
    self._halt = False
    

    self.cmd_pub = rospy.Publisher(NAV_TOPIC, AckermannDriveStamped, queue_size=100)
    print "[PID Control] using topic: %s"%pose_topic
    self.pose_sub =  rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)
  
  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):
    
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if 
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop

    # Suggested in code: remove elements that are behind the car, 
    # transform to cars coord system
    c=0
    while self.plan_idx < (self.plan.shape[0]):
      # transform to car coordinate system, translate by x_car, rotate (active) by theta_car
      # puts in coordinates where car points in +x direction, so test sign of plan_x
      theta_car = cur_pose[2]
      dist = np.sqrt(np.power((cur_pose[0] - self.plan[self.plan_idx,0]),2) + np.power((cur_pose[1] - self.plan[self.plan_idx,1]),2))
      x=self.plan[self.plan_idx,0] - cur_pose[0]
      y=self.plan[self.plan_idx,1] - cur_pose[1]
      R=np.array([[np.cos(theta_car), -1*np.sin(theta_car)],[ np.sin(theta_car), np.cos(theta_car)]])
      p=np.matmul(R.transpose(), np.array([x,y]))
      # grab x coordinate of pose in car frame, x < 0 implies car is infront of pose, drop
      # don't discard points that are far away..   add this to config discard_distance
      if p[0] < 0.0 and dist < 1.5:
          rospy.loginfo("Dropping pose from plan: %s"%str(self.plan[self.plan_idx]))
          #rospy.loginfo("  Cur Pose x: %s, y: %s, theta: %s"%(cur_pose[0], cur_pose[1], cur_pose[2]))
          #rospy.loginfo("  Plan x: %s, y: %s, theta: %s"%(self.plan[i][0], self.plan[i][1], self.plan[i][2]))

          self.plan_idx +=1
      else:  # only delete any initial plan nodes, since we may try to skip ahead
          break
      c+=1
      if c > 2*self.plan_lookahead:
        break

      
    # Check if the plan is empty. If so, return (Success, 0.0)
    if self.plan_idx == self.plan.shape[0]:
        self._halt=True
        rospy.loginfo("Have completed plan")
        return 0.0

    
    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index

    goal_idx = np.min((self.plan_idx+self.plan_lookahead, (self.plan.shape[0]-1)))
   
    # Compute the translation error between the robot and the configuration at goal_idx in the plan
    # euclid dist: translation_error  = np.sqrt(np.power(plan[goal_idx][0]-cur_pose[0], 2) + np.power(plan[goal_idx[1]-cur_pose[1],2))

    # Use waypoint method to generate an error term
    # Generate waypoint vector and perpendicular car unit vector
    # Note in this calculation, detla (steering angle) should be -alpha*werror
    c=np.array([np.cos(cur_pose[2] + np.pi/2), np.sin(cur_pose[2] + np.pi/2)])
    w=np.array([self.plan[goal_idx][0]-cur_pose[0], self.plan[goal_idx][1]-cur_pose[1]])
    werror=-np.dot(c,w)/np.sqrt(np.dot(w,w))
    #rospy.loginfo("debug: c.c: :%.3f, w.w: %.3f, c.w: %.3f, werror: :%.3f"%(np.dot(c,c), np.dot(w,w), np.dot(c,w), werror))

        
    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be carefult about the sign of the rotation error

    # Note the direction here.  
    rerror = -np.cos(self.plan[goal_idx][2]) + np.cos(cur_pose[2])
    error = self.ww * werror + self.rw * rerror
    # Log this to verify the issues w/ rotation errors
    #rospy.loginfo("Car Pose heading: %.3f"%cur_pose[2])
    #rospy.loginfo("Plan Pose heading: %.3f"%self.plan[goal_idx][2])
    #rospy.loginfo("Error: %.3f (note steering angle will be ~ -kp*error"%error)
    print "[PID] Error: [%s]\t\t\tRot error [%s] Heading error [%s]"%(str(error), str(rerror), str(werror))
    return error
    
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    deriv_error = 0
    if len(self.error_buff)>0:
        (e,t) = self.error_buff[-1]
        deriv_error = (error - e)/(now-t)
    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff
    
    # Add the current error to the buffer
    self.error_buff.append((error, now))
    
    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/


    # Note - no need to be accurate since this is a tunable param - 
    # I simply sum the values of the buffer and divide by time range
    integ_error=0
    i=0
    idx_end=len(self.error_buff) -1
    # not well checked, also arbitrary
    if idx_end > 5:
        t1=self.error_buff[0][1]
        t2=self.error_buff[-1][1]
        s=0.0
        for val in self.error_buff:
            s+= val[0]

        if t2>t1:
            ave=s/len(self.error_buff)
            integ_error = ave/(t2-t1)
        else:
            rospy.loginfo("t2 < t1 in compute error: t1: %s, e1: %s, t2: %s, e2: %s"%(t1,e1,t2,e2))

    if self._i % 5 == 0:
        rospy.loginfo("Error calculations: deriv_error: %s; integ_error: %s"%(deriv_error, integ_error))
    delta = -self.kp*error + self.ki * integ_error + self.kd * deriv_error
    #rospy.loginfo("Steering angle delta: %.3f, (err: %.3f, dererr: %.3f, interr: %.3f"%(delta, error, deriv_error, integ_error))
    return delta
    
  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''  
  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         Utils.quaternion_to_angle(msg.pose.orientation)])
    
    error = self.compute_error(cur_pose)
    # 
    self._i+=1
    if self._halt:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      return
      
    delta = self.compute_steering_angle(error)
    print "[PID] steering delta is: %s"%delta
    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)

# Having a lot of time updating the initial pose
def update_initial_pose():
  
  initial_pose_sub =  rospy.Publisher(INITIALPOSE_TOPIC, PoseWithCovarianceStamped, queue_size=1)
  point = Point(x=10.0, y=10.0)
  q = Utils.angle_to_quaternion(0.9)
  pose = PoseWithCovariance(pose=Pose(position=Point, orientation=q))
  initial_pose_sub.publish(PoseWithCovarianceStamped(pose=pose))
  rospy.loginfo("Publishing initial pose, Pose: %s, Orientation: %s"%(point, q))

def main():

  rospy.init_node('line_follower', anonymous=True) # Initialize the node


  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LineFollower class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system
  # YOUR CODE HERE
  plan_topic = PLAN_TOPIC
  pose_topic = POSE_TOPIC # Default val: '/car/pose'
  plan_lookahead = rospy.get_param("~plan_lookahead", 3) # Starting val: 5
  waypoint_weight = rospy.get_param("~waypoint_weight", 1.0) # Starting val: 1.0
  rotation_weight = rospy.get_param("~rot_weight", 0.0) # Starting val: 0.0
  kp = rospy.get_param("~kp", 1.0) # Startinig val: 1.0
  ki = rospy.get_param("~ki", 0.0) # Starting val: 0.0
  kd = rospy.get_param("~kd", 0.0) # Starting val: 0.0
  error_buff_length = rospy.get_param("~error_buff_len", 10) # Starting val: 10
  speed = rospy.get_param("~speed", 1.0) # Default val: 1.0


  raw_input("Press Enter when plan available...")  

  rospyplan = rospy.wait_for_message(PLAN_TOPIC, PoseArray)
  plan = []
  for pose in rospyplan.poses:
      #rospy.loginfo("Pose: x: %s , y: %s, theta: %s"%(pose.position.x, pose.position.y, utils.quaternion_to_angle(pose.orientation)))
      plan.append((pose.position.x, pose.position.y, Utils.quaternion_to_angle(pose.orientation)))

  raw_input("Press Enter to run plan...") 

  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  #lf = LineFollower()
  lf = LineFollower( plan, pose_topic, plan_lookahead, waypoint_weight,
                     rotation_weight, kp, ki, kd, error_buff_length, speed)

  
  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()
