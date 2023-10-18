#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock
import sys 

from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray
# service container objects
from final.srv import GetPlan, GetPlanResponse, AddWaypoint, AddWaypointResponse, Reset, ResetResponse

from HaltonPlanner import HaltonPlanner
from HaltonEnvironment import HaltonEnvironment
import GraphGenerator
import Utils


# Global topcs
ADDWAYPOINT_SERVICE = "planner_node/add_waypoint"
GETPLAN_SERVICE = "planner_node/get_plan"
RESET_SERVICE = "planner_node/reset"

class PlannerNode(object):

  

  def __init__(self, map_service_name, 
                     halton_points, 
                     disc_radius,
                     collision_delta,                      
                     source_topic,
                     target_topic,
                     pub_topic,
                     service_topic,
                     car_width,
                     car_length):
    
    print("[Planner Node] Getting map from service...")
    rospy.wait_for_service(map_service_name)
    self.map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
    print("[Planner Node] ...got map")
    
    print("[Planner Node] Generating graph file...")
    graph_file = GraphGenerator.generate_graph_file(self.map_msg, halton_points, disc_radius, car_width, car_length, collision_delta)
    print("[Planner Node] ..graph generated")
    
    self.environment = HaltonEnvironment(self.map_msg, graph_file, None, None, car_width, car_length, disc_radius, collision_delta)
    self.planner = HaltonPlanner(self.environment)
    
    self.source_pose = None
    self.source_yaw = None
    self.source_lock = Lock()
    self.target_pose = None
    self.target_yaw = None
    self.target_lock = Lock()
    self.cur_plan = None
    self.plan_lock = Lock()
    self.nplans = 0
    
    self.orientation_window_size = 4

    print ('[Planner Node] starting service with topic: ', GETPLAN_SERVICE)
    self.plan_service = rospy.Service(GETPLAN_SERVICE, GetPlan, self.get_plan_cb)
    
    print ('[Planner Node] starting service with topic: ', ADDWAYPOINT_SERVICE)
    self.addwaypoint_service = rospy.Service(ADDWAYPOINT_SERVICE, AddWaypoint, self.add_waypoint_cb)

    print ('[Planner Node] starting service with topic: ', RESET_SERVICE)
    self.reset_service = rospy.Service(RESET_SERVICE, Reset, self.reset_cb)

    print '[Planner Node] Ready to plan'
    """
    if pub_topic is not None:
      self.plan_pub = rospy.Publisher(pub_topic, PoseArray, queue_size=1)  
      self.source_sub = rospy.Subscriber(source_topic, 
                                         PoseWithCovarianceStamped, 
                                         self.source_cb,
                                         queue_size=1)
      self.target_sub = rospy.Subscriber(target_topic, 
                                         PoseStamped, 
                                         self.target_cb,
                                         queue_size=1)          
    else:
      self.plan_pub = None
    """
    
  ## Call back to reset the plan
  def reset_cb(self, req):
    self.source_lock.acquire()
    self.source_pose = None
    self.source_yaw = None
    self.source_lock.release()    
    self.target_lock.acquire()
    self.target_pose = None
    self.target_yaw = None
    self.target_lock.release()  
    self.nplans = 0
    self.cur_plan = None
    print "[Planner Node] reset all planning states"
    resp = ResetResponse()
    resp.success = True
    return resp
    
  ## Call back to add a waypoint to the plan
  # 
  #  req.waypoint:  float32[]
  def add_waypoint_cb(self, req):
    print '[Planner Node] add waypoint service request'
    update = False
    # first waypoint is the source
    if self.source_pose is None:
      self.source_lock.acquire()
      self.source_pose = [req.waypoint[0], req.waypoint[1]]
      self.source_yaw = 0.0 if len(req.waypoint)==2 else req.waypoint[2]
      self.source_lock.release()    
    # second waypoint is the target
    elif self.target_pose is None:
      self.target_lock.acquire()
      self.target_pose = [req.waypoint[0], req.waypoint[1]]
      self.target_yaw = 0.0 if len(req.waypoint) == 2 else req.waypoint[2]
      self.target_lock.release()  
      update = True
    # subsequent waypoint calculates a new path.
    else: 
      self.source_lock.acquire()
      self.source_pose = self.target_pose
      self.source_yaw = self.target_yaw
      self.source_lock.release() 
      self.target_lock.acquire()
      self.target_pose = [req.waypoint[0], req.waypoint[1]]
      self.target_yaw = 0.0 if len(req.waypoint) == 2 else req.waypoint[2]
      self.target_lock.release()  
      update = True

    # Update plan
    if update:
      self.plan_lock.acquire()
      self.update_plan()
      self.plan_lock.release() 
    resp = AddWaypointResponse()
    print '[Planner Node] generating response'
    
    resp.success = False if self.cur_plan is None else True
    return resp

  ## Call back to initialize a plan:
  #
  #  req:                   Not used
  #  resp.succcess: bool    
  #  resp.plan: float32[]   Plan flattened to a 1d array
  #
  def get_plan_cb(self, req):
    print '[Planner Node] get plan service request'
    resp = GetPlanResponse()
    print '[Planner Node] generating response'
    
    resp.success = False if self.cur_plan is None else True
    if resp.success:
      if req.smooth:  
        print "[Planner Node] smoothing plan"
        self.smooth_plan(self.cur_plan)
      resp.plan = self.flatten(self.cur_plan)
    return resp
    
  def flatten(self, plan):
    flatplan = [item for sublist in plan for item in sublist]
    return flatplan
    
 
      
  # It would be really nice to have some better documentation on how the skeleton code is connected.
  # plan is a list of lists: [[x1,y1], [x2, y2], ... ]
  def add_orientation(self, plan):
    plan = np.array(plan)

    oriented_plan = np.zeros((plan.shape[0],3),dtype=np.float)
    oriented_plan[:,0:2] = plan[:,:]
 
    if plan.shape[0] >= 2:  
      # our plan does this.  Planner
      initial_yaw = np.arctan2(plan[1,1]-plan[0,1],plan[1,0]-plan[0,0])
      final_yaw = np.arctan2(plan[-1,1]-plan[-2,1],plan[-1,0]-plan[-2,0])
      oriented_plan[0,2] = initial_yaw
      oriented_plan[oriented_plan.shape[0]-1, 2] = final_yaw
          
      plan_diffs = np.zeros(plan.shape, np.float)
      plan_diffs[0:plan_diffs.shape[0]-1] = plan[1:plan.shape[0]]-plan[0:plan.shape[0]-1]
      plan_diffs[plan_diffs.shape[0]-1] = np.array([np.cos(self.target_yaw), np.sin(self.target_yaw)], dtype=np.float) 
    
      avg_diffs = np.empty(plan_diffs.shape, dtype=np.float)
      for i in xrange(plan_diffs.shape[0]):
        avg_diffs[i] = np.mean(plan_diffs[np.max((0,i-self.orientation_window_size/2)):
                                          np.min((plan_diffs.shape[0]-1, i+self.orientation_window_size/2+1))],
                               axis=0)

      oriented_plan[1:oriented_plan.shape[0]-1,2] = np.arctan2(avg_diffs[1:oriented_plan.shape[0]-1,1],
                                                               avg_diffs[1:oriented_plan.shape[0]-1,0])
   
    elif plan.shape[0] == 2:
      oriented_plan[:,2] = np.arctan2(plan[1,1]-plan[0,1],plan[1,0]-plan[0,0])

    return oriented_plan
  
  # takes angular average of orientations before/after current point
  def smooth_plan(self, plan, num=2):
    yaw=np.zeros(plan.shape[0])
    for i in range(plan.shape[0]):
      idx0=np.max((0,i-num))
      idx1=np.min((plan.shape[0],i+num))
      n=idx1-idx0
      s=np.sum(np.sin(plan[idx0:idx1,2]))/n
      c=np.sum(np.cos(plan[idx0:idx1,2]))/n
      theta = np.arctan(s/c)
      if c <  0:
        theta += np.pi
      yaw[i]=theta
    plan[:,2]=yaw
    
      
  ## Calls planner.plan()
  #
  #  planner.plan() => [[x1, y1], [x2, y2], ... [xn, yn]]
  #  mutates plan (into ndarray w shape (n,3)) with orientation/yaw
  def update_plan(self):
    self.source_lock.acquire()
    self.target_lock.acquire()
    if self.source_pose is not None:
      source_pose = np.array(self.source_pose).reshape(2)
    if self.target_pose is not None:
      target_pose = np.array(self.target_pose).reshape(2)
    plan = self.source_pose is not None and self.target_pose is not None
    self.source_lock.release()
    self.target_lock.release()
    
    if plan:
      if(np.abs(source_pose-target_pose).sum() < sys.float_info.epsilon):
        print '[Planner Node] Source and target are the same, will not plan'
        return
      
      if not self.environment.manager.get_state_validity(source_pose):
        print '[Planner Node] Source in collision, will not plan'
        return
      
      if not self.environment.manager.get_state_validity(target_pose):
        print '[Planner Node] Target in collision, will not plan'
        return

      if self.nplans == 0:
        # This is crucial step, last 2 nodes set up are expected to source and target
        print '[Planner Node] Inserting source and target'
        self.environment.set_source_and_target(source_pose, target_pose)
      else: 
        print '[Planner Node] Adding new atarget'
        self.environment.add_new_target(target_pose)
      self.nplans+=1

      # Calls plan algorithm
      print '[Planner Node] Computing plan...'      
      plan = self.planner.plan()    
      
      if self.planner.success:
        
        plan = self.add_orientation(plan)
        # smoothing angles is probably not good solution.  Better will be to adjust way points to create arc for sharp turns
        #self.smooth_plan(plan)

        print '[Planner Node] path length: %s'%self.planner.get_pathlength(plan)
        if self.cur_plan is None:  # first time
          self.cur_plan = plan
        else:
          self.cur_plan = np.vstack((self.cur_plan, plan))

        print '[Planner Node] skipping running post process'
        # self.planner.visualize(self.cur_plan, pre=False)  ## skipping pre processing
        # self.cur_plan = self.planner.post_process(self.cur_plan, 5)
        # print '[Planner Node] post process complete'
        # print '[Planner Node] path length: %s'%self.planner.get_pathlength(self.cur_plan)
        #print '[Planner Node] ...plan complete (visualizing)'
        #self.planner.visualize(self.cur_plan, pre=False)
        
      else:
        print '[Planner Node] ...could not compute a plan'
      
    # In final project - let driver control publishing plan
    #if (self.cur_plan is not None) and (self.plan_pub is not None):
    #  print "[Planner Node] publishing new plan"
    #  self.publish_plan(self.cur_plan)  
    
if __name__ == '__main__':
  rospy.init_node('planner_node', anonymous=True)
  
  map_service_name = rospy.get_param("~static_map", "static_map")
  halton_points = rospy.get_param("~halton_points", 500)
  disc_radius = rospy.get_param("~disc_radius", 3)
  collision_delta = rospy.get_param("~collision_delta", 0.05)  
  source_topic = rospy.get_param("~source_topic" , "/initialpose")
  target_topic = rospy.get_param("~target_topic", "/move_base_simple/goal")
  pub_topic = rospy.get_param("~pub_topic", None)
  service_topic = rospy.get_param("~service_topic", None)
  car_width = rospy.get_param("/car_kinematics/car_width", 0.33)
  car_length = rospy.get_param("/car_kinematics/car_length", 0.33)

  
  pn = PlannerNode(map_service_name, 
                   halton_points, 
                   disc_radius,
                   collision_delta,                    
                   source_topic,
                   target_topic,
                   pub_topic,
                   service_topic,
                   car_width,
                   car_length)
                   
  rospy.spin()
