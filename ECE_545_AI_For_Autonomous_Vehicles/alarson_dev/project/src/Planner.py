#!/usr/bin/env python

import rospy
import numpy as np
from final.srv import *
import PlannerNode 
from LineFollower import LineFollower
import Utils
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray

# simple utility class that can save plans and load plans

"""
  Write plan to file in csv format
  "1.2,3.4,5.6\n"

  NOTE:  cwd for saving/reading plans is ~/.ros
"""  
def save_plan(plan, filename):
  print 'saving plan with shape: %s'%str(plan.shape)
  f = open(filename, "w")
  for i in range(plan.shape[0]):
    row = plan[i]
    f.write("%s,%s,%s\n"%(str(row[0]),str(row[1]), str(row[2])))
  f.close()


"""
  Load a plan from file
  Expects lines with format "1.2,3.4,5.6\n", does no error handling
  returns np array with shape [N,3]

  NOTE:  cwd for saving/reading plans is ~/.ros
"""
def load_plan(filename):
  print 'loading plan %s'%filename
  f = open(filename, "r")
  plan = []
  for line in f.readlines():
    plan.append(map(np.float32, line.strip().split(",")))
  f.close()

  return np.array(plan)

# Way points for small_basement map
# waypoints = np.array([[2500, 640, 0.0], [2600, 660, 0.0],[1880, 450, 0.0], [1435, 545, 0.0], [1250, 460, 0.0], [540, 835, 0.0]])
# shorter testing version

#waypoints = np.array([[2500, 640, 0.0],  [1880, 450, 0.0], [1880, 250, 0.0], [1400, 250, 0.0]])
#waypoints = np.array([[2500, 640, 0.0], [2575, 640, 0.0], [2600, 660, 0.0], [2575, 680, 0.0], [2400, 700, 0.0]])
# initial turn
p = [
  [2500, 640, 0.0], [2510,580,0.0], [2550, 560, 0.0], [2590, 600, 0.0],
  [2600, 660, 0.0], [2600, 690, 0.0], [1880, 440, 0.0], [1880, 250, 0.0]
]
waypoints = np.array(p)
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

  rospy.init_node('plan_node', anonymous=True)
  rate=rospy.Rate(10)
  map_service_name = rospy.get_param("~static_map", "static_map")
  print("Getting map from service: ", map_service_name)
  rospy.wait_for_service(map_service_name)
  map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
  print("Got map")
  
  print "[Planner] Waiting for GetPlan service"
  rospy.wait_for_service(PlannerNode.GETPLAN_SERVICE)
  get_plan = rospy.ServiceProxy(PlannerNode.GETPLAN_SERVICE, GetPlan)
  print "[Planner] Waiting for AddWaypoint service"
  rospy.wait_for_service(PlannerNode.ADDWAYPOINT_SERVICE)
  add_waypoint = rospy.ServiceProxy(PlannerNode.ADDWAYPOINT_SERVICE, AddWaypoint)
  print "[Planner] Waiting for Reset service"
  rospy.wait_for_service(PlannerNode.RESET_SERVICE)
  reset = rospy.ServiceProxy(PlannerNode.RESET_SERVICE, Reset)
  reset(True)
  
  # Plan Publisher
  plan_topic = rospy.get_param("~plan_topic", "driver_node/car_plan")
  plan_pub = rospy.Publisher(plan_topic, PoseArray, queue_size=100)  

  
  try:
    for i in range(waypoints.shape[0]):
      w=waypoints[i]
      print("Calling service for add waypoint (%s)"%str(w))
      waypoint = Utils._map_to_world(w, map_info)
      print("In world waypoint is (%s)"%str(waypoint))
      resp = add_waypoint(waypoint)


    resp = get_plan(True)
    plan = np.array(resp.plan).reshape(-1, 3) 
    print "Get Plan returned: %s, plan shape = %s"%(resp.success, str(plan.shape))

  except rospy.ServiceException, e:
    print 'Service call failed: %s' % e
  
  print "full plan shape: %s"%str(plan.shape)
  
  
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
  
  plan_pub.publish(pa) 
  print "published plan to %s"%plan_topic
  #filename = rospy.get_param("~plan_file", "test.csv")
  filename = "initial_turn2.csv"
  print "writing plan to file: %s"%filename
  save_plan(plan, filename)
  print "Done"
  rospy.spin()
