#!/usr/bin/env python

import rospy
import numpy as np
from final.srv import *
import PlannerNode 
from LineFollower import LineFollower
import Planner
import Utils
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray


# Way points for small_basement map
#waypoints = np.array([[2500, 640, 0.0], [2600, 660, 0.0],[1880, 450, 0.0], [1435, 545, 0.0], [1250, 460, 0.0], [540, 835, 0.0]])
# shorter testing version
#waypoints = np.array([[2500, 640, 0.0], [2600, 660, 0.0], [1880, 450, 0.0]])
#waypoints = np.array([[2500, 640, 0.0], [2580, 680, 0.0], [2600, 660, 0.0], [2580, 640, 0.0], [2400, 460, 0.0]])
#waypoints = np.array([[2500, 640, 0.0], [1880, 450, 0.0]])
# Add bad way points


# Plan publisher
# plan [[x,y,theta], ... [xn,yn,thetan]]
def publish_plan(pub, plan):
  pa = PoseArray()
  pa.header.frame_id = "/map"
  for i in xrange(len(plan)):
    print "publishing plan point: %s"%str(plan[i])
    config = plan[i]
    pose = Pose()
    pose.position.x = config[0]
    pose.position.y = config[1]
    pose.position.z = 0.0
    pose.orientation = Utils.angle_to_quaternion(config[2])
    pa.poses.append(pose)
  pub.publish(pa) 
  print "published plan"


if __name__ == '__main__':

  rospy.init_node('driver_node', anonymous=True)
  rate=rospy.Rate(10)
  
  map_service_name = rospy.get_param("~static_map", "static_map")
  print("Getting map from service: ", map_service_name)
  rospy.wait_for_service(map_service_name)
  map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
  print("Got map")
 
  # publish plan
  plan_topic = rospy.get_param("~plan_topic", None)
  plan_pub = rospy.Publisher(plan_topic, PoseArray, queue_size=100)  
  viz_plan_topic = rospy.get_param("~viz_plan_topic", None)
  viz_plan_pub = rospy.Publisher(viz_plan_topic, PoseArray, queue_size=100)  

  # loads plan from file, should be (N,3) np.array
  plan_file = rospy.get_param("~planner_file","test.csv")
  # Note expects plans to be found in ~/.ros
  #plan_file="simple_plan.csv"
  plan_file="initial_turn2.csv"
  print "[Driver] loading plan %s: "%plan_file
  plan = Planner.load_plan(plan_file)
  print "[Driver] full plan shape: %s"%str(plan.shape)
  rospy.sleep(2)
  
  pa = PoseArray()
  pa.header.frame_id = "/map"
  for i in xrange(len(plan)):
    config = plan[i]
    pose = Pose()
    pose.position.x = config[0]
    pose.position.y = config[1]
    pose.position.z = 0.0
    pose.orientation = Utils.angle_to_quaternion(config[2])
    #print "Appending pose to plan: x:%s, y:%s"%(pose.position.x, pose.position.y)
    pa.poses.append(pose)
  
  viz_plan_pub.publish(pa)
  plan_pub.publish(pa) 
  print "published plan to %s"%plan_topic
  #rate.sleep()
  #while not rospy.is_shutdown():
  #  rospy.sleep(1)

  raw_input("Have a plan.  Press Enter to run plan...") 

  # Launch line follower with parameters in launch file
  pose_topic = rospy.get_param("~pose_topic", "/inferred_pose") # Default val: '/car/pose'
  plan_lookahead = rospy.get_param("~plan_lookahead", 3) # Starting val: 5
  waypoint_weight = rospy.get_param("~waypoint_weight", 1.0) # Starting val: 1.0
  rotation_weight = rospy.get_param("~rot_weight", 0.0) # Starting val: 0.0
  kp = rospy.get_param("~kp", 1.0) # Startinig val: 1.0
  ki = rospy.get_param("~ki", 0.0) # Starting val: 0.0
  kd = rospy.get_param("~kd", 0.0) # Starting val: 0.0
  error_buff_length = rospy.get_param("~error_buff_len", 10) # Starting val: 10
  speed = rospy.get_param("~speed", 1.0) # Default val: 1.0
  
  lf = LineFollower( plan, pose_topic, plan_lookahead, waypoint_weight,
                     rotation_weight, kp, ki, kd, error_buff_length, speed)


  # if wanted a state manager to peridoically muated objects, could be accomplished here.
  # likely that the particle filter will live here.
  # rospy.spin()
  while not rospy.is_shutdown(): # Keep going until we kill it
    rospy.sleep(1)
