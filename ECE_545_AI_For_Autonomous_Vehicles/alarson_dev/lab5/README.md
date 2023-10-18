## Lab 5 - Implementing A* to find valid path between points on map

- Arnie Larson, Jordan Fraser, Sanjar Normuradov
- 11/24/2022
- Group 11 (car 33)


### Todo ###

- [*] Implement A*
- [*] Implement Obstacle Manager discritize edges
- [*] Figure out/fix PlannerNode service
- [*] Figure out visualizations
- [*] Implement check state validity (wall collision)
- [*] Implement post processing (path improvement with random process)
- [*] Finish write up


### Assignment Criteria

- Submit visualization for each of the source/target pairs in planner_test
  - What is the path length for each?
  - Tartet 1: (5 nodes in solution)
  - Test 1: Pathlength went from 10.25 to 9.87
  - Test 2: Pathlength went from 18.58 to 17.92
  - Test 3: Pathlenght went form 39.23 to 38.5
- Submit the first target pair, show the path without post processing
  - How are the paths different
  - For the first target, their are only 5 nodes in the plan, and the node vertices are clearly visible in the plan.
- Modify Halton Points and disc_radius settings, repeat steps 1 and 2 and discuss difference
  - I increased the number of points and made the radius smaller (2250, 2)
    - Expectation is that there will be almost no difference, but the initial path (without post processing) should be closer to the exact path.  However, number of edges will explode and so algorithm time will also go up significantly.
    - Target 1: (7 nodes in solution) 
    - Target 1: Pathlength went from 10.25 to 9.82
    - Target 2: Pathlength went from 18.3 to 17.92
    - Target 3: Pathlenght went form 39.23 to 38.5
  - I decreased the number of points and made the radius bigger (750, 5)
    - Expectation is that as long as a solution is found with equivalent topology, then post processing will converge to a "ideal" solution.  Compute time is definitely shorter with the shorter graph.
    - Target 1: (now only 4 nodes in solution) 
    - Target 1: Pathlength went from 10.6 to 9.86
    - Target 2: Pathlength went from 18.88 to 17.93
    - Target 3: Pathlenght went form 39.3 to 38.47
  - Lazy A*
    - Since compuation is in the edge checking, it could be great to find a solution then backtrack.
    - Would need to be able to validate edges along solution.  
      - If a bad edge is found.  Remove all of the next parent/child relationships in solution
      - Presumably we could invalidate the node with the bad edge. 
        - This could have multiple side effects..  that would have to be handled.
        - 1. What if this was the target node? 
        - 2. What if this node is a required node in the path?  Then solution would be inaccessible
        - 3. What if this node was a useful node in the path?  Then the found path would be different/wonky


### Overview

THere is a lot of starter code and utility code here, our main goal is to implement A* to find a good path between points.  
The started code includes code that generates a graph (using networkx package), and a planner node which ultimately calls the get_plan function which hooks into our code.  The code that generates the connection between planner_test and PlannerNode is somewhat broken and requires updates to get to work.  

Our main code hook is in the planner (HaltonPlanner.py) and searches for a solution.  This extracts a start and end node and searches nearest neighborhoods using a cost that is node distance + distance to target/end node.


### A*

Just like Dijkstra algorithm, but uses a heristic to move towards the end goal faster.

nodes:
  start 
  end
closed = {node}   # set of nodes that have been reached already (or are invalid)
mins = {node: cost}  # track cost to get to node
parents = {parents:child}  # tracks best parent node for a given child node


Initialize priority queue q with (0+g(n), start)
while q:
  (cost, node) = q.pop
  if node is end:
    return # done, set success flag
  
  add node to closed
  # get neighbors, update queues and paths
  for n in neighbors:
    if n is in closed:
      continue
    # is n already in mins?  if so it's valid
    if n is not valid:
      add to closed
      continue
    # check edge validity between current node and n
    if edge is not valid:
      continue
    # compute costs and update priority queue
    ncost = cost + f(n) + g(n) 
    if n not in mins or ncost < mins.get(n): # update graphs variables
        mins[n] = ncost
        parents[n] = node
        q.push((ncost, n))

At this point either a solution has been found OR the end node was not reachable (consider maintaining a state variable)
        
### Object collision detection

For each new possible node - we should first check if it's a valid location (via collision checking), and close it if not.  Then we should check that the edge between current and next node is valid.  

Algorithms
  - given a central pose (x,y), could either:
    - check each point within a bounding box (x-.5W, y-.5L) to (x+.5W, y + .5L)
    - **initial choics** could check the boundary points
    - could check some random set of points within the boundary
  - to check edge validity, algorithm is
    - discretize edge into N points [Start to End)
    - at each point check bounding box


### Lazy A*

Constraint in A* is that although we don't check so many nodes, we do check a lot of edges, and that takes the most compute time.  Can we delay validating edges until we are composing the solution?

Idea:
  - Since compuation is in the edge checking, it could be great to find a solution then backtrack.
  - Would need to be able to validate edges along solution.  
    - If a bad edge is found.  Remove all of the next parent/child relationships in solution
    - Easiest implementation we could invalidate the node with the bad edge and restart the algorithm from the beginnig. 
      - This could have multiple side effects..  that would have to be handled.
      - 1. What if this was the target node? 
      - 2. What if this node is a required node in the path?  Then solution would be inaccessible
      - 3. What if this node was a useful node in the path?  Then the found path would be different/wonky