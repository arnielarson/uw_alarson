import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random
import rospy
from heapq import *

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan (flat plan - sequence of x,y pairs)
  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id
    rospy.loginfo("Source id, sid: %s"%self.sid)
    rospy.loginfo("Target id, tid: %s"%self.tid)

    ## WHy does the skeleton code use object members here? 
    
    self.closed = set() # The closed list
    self.parent = {self.sid:None} # A dictionary mapping children to their parents (tracks the paths)
    self.mins = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.success = False
    # is this useful?
    self.cost = 0
    edges = set()
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors()
    # - self.planningEnv.get_distance()
    # - self.planningEnv.get_heuristic()
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------
    

    # Start at start, use priority queue to keep track of f(n) = cost(n) + h(n)
    # In class example - graph is {node: (neighbor, cost) }
    # In our example, can get neighbors, and compute costs from get_distance and get_heuristic
    
    q = []
    count = 1
    nedges = 0
    nvalids = 0
    heappush(q, (0 + self.planningEnv.get_heuristic(self.sid, self.tid), self.sid))
    while q:

      (cost, curnode) = heappop(q)
      if curnode == self.tid:
        print "found target, cost was %s"%cost
        self.success = True
        # tid cost and parent graphs has already been added in for loop
        break
      if curnode not in self.closed:  # add it to closed list
        self.closed.add(curnode)
      
      neighbors = self.planningEnv.get_successors(curnode)
      for n in neighbors:
        
        if n in self.closed:
          continue
        if n not in self.mins:
          nvalids += 1  # only have to check each node once here
          if not self.planningEnv.manager.get_state_validity(self.planningEnv.get_config(n)):
            self.closed.add(n)
            continue
        # invalidates this path, not the node
        nedges += 1  # have to check each edge for validity
        if not self.planningEnv.manager.get_edge_validity(self.planningEnv.get_config(curnode), self.planningEnv.get_config(n)):
          continue        
        
        ncost = cost + self.planningEnv.get_distance(curnode, n) + self.planningEnv.get_heuristic(n, self.tid)        
        # update search space and global variables
        if n not in self.mins or ncost < self.mins.get(n):
          count += 1
          self.parent[n]=curnode
          self.mins[n] = ncost
          heappush(q, (ncost, n))
        
    print "generated solution, checked %s nodes"%count
    print "generated solution, checked %s edges"%nedges
    print "generated solution, checked %s valids"%nvalids

    # this is used in get_solution - not sure what's its purpose is?
    self.planIndices = []
    plan = self.get_solution(self.tid) if self.success else []
    return plan

  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  # plan is [[x1, y1], [x2,y2], ... ]
  def post_process(self, plan, timeout):
    print "[Planner] plan shape: %s"%str(plan.shape)
    t1 = time.time()
    elapsed = 0
    updates = 0
    while elapsed < timeout: # Keep going until out of time
      size = plan.shape[0]
      draws = numpy.random.randint(0,size, 2)
      i = draws.min()
      j = draws.max()
      if i == j:
        continue

      config1 = plan[i,:]
      config2 = plan[j,:]
      valid, lx, ly = self.planningEnv.manager.update_path(config1, config2)

      if valid:
        updates +=1
        #print "[Planner] post process updating plan at index %s to %s"%(i,j)
        theta = numpy.arctan((ly[1]-ly[0])/(lx[1]-lx[0]))
        config = numpy.zeros((len(lx), 3))
        config[:,0] = lx
        config[:,1] = ly
        config[:,2] = theta
        # insert into plan, (leaves in end point)
        plan = numpy.delete(plan, numpy.arange(i,j), axis=0)
        plan = numpy.insert(plan, i, config, axis=0)  # insertion should be along the 0 axis (rows) as those are the time steps
      
      elapsed = time.time() - t1
    print "[Planner] plan shape: %s"%str(plan.shape)
    print "[Planner] made %s updates"%updates
    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]
      

    plan = []
    planID.reverse()
    ## Debug code:
    for node in planID:
      pose = self.planningEnv.get_config(node)
      print("Node: %s, x: %s, y: %s"%(node, pose[0], pose[1]))

    for i in range(len(planID) - 1):
      print "Adding graph node to plan: %s"%planID[i]
      startConfig = self.planningEnv.get_config(planID[i])  # get_config returns x,y coords
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      # This is basically adding [[px, py]] for all elements in px, py (which are iterables)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    # plan is a list of lists of lists.  This flattens it to a list of lists 
    # 
    flatPlan = [item for sublist in plan for item in sublist]
    print "len flat plan: %s"%len(flatPlan)
    print "flatplan[0]: %s"%flatPlan[0]
    
    # At this point can visualize the plan
    #self.visualize(flatPlan, pre=True)


    #for l in flatPlan:
    #  print "flat plan element: %s"%l
    return flatPlan

  def get_pathlength(self, plan):
    D=0.0
    for i in range(plan.shape[0]-1):
      D+= numpy.sqrt(numpy.power(plan[i+1,0]-plan[i,0],2) + numpy.power(plan[i+1,1]-plan[i,1],2))
    return D


  # Visualize the plan
  #   Called from HaltonPlanner, passes in [[x,y],..] type plan
  #   Called from PlannerNode, passes in np array(N,3), [x,y,theta]
  def visualize(self, plan, pre=True):
    if pre:
      plan = numpy.array(plan)
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    #for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
    for i in range(plan.shape[0]-1):
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)
    ri = numpy.random.randint(1,100000)
    cv2.imwrite("/home/robot/catkin_ws/src/lab5/pathviz_%s_small_graph_.png"%ri, envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
