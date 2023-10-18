#!/usr/bin/env python
# Student: Arnold Larson
# Class: UW EEP 545

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
import Utils 

SUB_TOPIC = '/car/car_pose' # The topic that provides the simulated car pose
PUB_TOPIC = '/clone/pose' # The topic that you should publish to
MAP_TOPIC = '/static_map' # The service topic that will provide the map


class CloneFollower:

  '''
  Initializes a CloneFollower object
  In:
    follow_offset: The required x offset between the robot and its clone follower
    force_in_bounds: Whether the clone should toggle between following in front
                     and behind when it goes out of bounds of the map
  '''
  def __init__(self, follow_offset, force_in_bounds):

    self.nmsg = 0
    self.follow_offset = float(follow_offset)
    self.force_in_bounds = bool(force_in_bounds)
    rospy.loginfo("Initializing Clone Follower")
    rospy.loginfo("Clone Follower  Params: offset: %s; force_in_bounds: %s"%(self.follow_offset, self.force_in_bounds))

    self.map_img, self.map_info = Utils.get_map(MAP_TOPIC)
    rospy.loginfo("Getting map: width: %s; height: %s; resolution: %s"%(self.map_info.width, self.map_info.height, self.map_info.resolution))
    ## Explore map:
    try:
        rospy.loginfo("map shape: (%s,%s)"%(self.map_img.shape))
        rospy.loginfo("map value [0,0]: %s"%self.map_img[0][0])
        rospy.loginfo("map value [400,400]: %s"%self.map_img[400][400])
    except Exception as e:
        rospy.loginfo("Exception %s"%e.message)
    
    self.pub = rospy.Publisher(PUB_TOPIC, PoseStamped, queue_size=100)
    
    self.sub =  rospy.Subscriber(SUB_TOPIC, PoseStamped, self.update_pose)
    
  '''
  Given the translation and rotation between the robot and map, computes the pose
  of the clone
  (This function is optional)
  In:
    trans: The translation between the robot and map
    rot: The rotation between the robot and map
  Out:
    The pose of the clone
  '''
  def compute_follow_pose(self, trans, rot):

    return
    
  '''
  Callback that runs each time a sim pose is received. Should publish an updated
  pose of the clone.
  In:
    msg: The pose of the simulated car. Should be a geometry_msgs/PoseStamped
  '''  
  def update_pose(self, msg):
    self.nmsg+=1 # stupid debugging trick
    x=msg.pose.position.x
    y=msg.pose.position.y
    z=msg.pose.position.z
    theta=Utils.quaternion_to_angle(msg.pose.orientation)
    cx= x + self.follow_offset*np.cos(theta)
    cy= y + self.follow_offset*np.sin(theta)
    cz= z

    # Check bounds if required
    if self.force_in_bounds:
      board = Utils.world_to_map((cx,cy,theta), self.map_info)
      valid = self.map_img[board[0]][board[1]]

      if self.nmsg%50 == 0:
          rospy.loginfo("x, y: (%s,%s);   board i,j: (%s,%s);  valid: %s"%(cx,cy,board[0],board[1],valid))

      if not valid:
          self.follow_offset *= -1

      # pass in (x,y,theta); map_info, computes map coords
      # get value of map at x,y
      # Functions in Utils.py will again be useful here

    update = msg
    update.pose.position.x= x + self.follow_offset*np.cos(theta)
    update.pose.position.y= y + self.follow_offset*np.sin(theta)
    update.pose.position.z= z
    update.pose.orientation = msg.pose.orientation  # unnecessary?

    self.pub.publish(update)
    return
      
    
if __name__ == '__main__':

  # todo: Get params from launch script, NOTE, COULDN'T GET THIS TO WORK
  # testing
  #p1=rospy.get_param("clone_offest"); p2=ro spy.get_param("~clone_offset")
  #print("TESTING:  p1: %s; p2: %s"%(p1,p2))
  # rospy.get_param("~param_name", default)
  
  rospy.init_node('clone_follower', anonymous=True) # Initialize the node
  follow_offset = float(rospy.get_param("~clone_offset", "-1.7"))
  force_in_bounds = bool(rospy.get_param("~clone_in_bounds", "1"))
  print("Clone offset: %s;  force in bounds: %s "%(follow_offset, force_in_bounds))
  
  cf = CloneFollower(follow_offset, force_in_bounds) # Create a clone follower
  rospy.spin() 
  
