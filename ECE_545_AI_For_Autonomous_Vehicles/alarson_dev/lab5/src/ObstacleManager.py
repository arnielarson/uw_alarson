import cv2
import math
import numpy as np
import Utils

class ObstacleManager(object):

  def __init__(self, mapMsg, car_width, car_length, collision_delta):
    self.map_info = mapMsg.info
    self.mapImageGS = np.array(mapMsg.data, dtype=np.uint8).reshape(
            (mapMsg.info.height, mapMsg.info.width, 1))

    # Retrieve the map dimensions
    height, width, channels = self.mapImageGS.shape
    self.mapHeight = height
    self.mapWidth = width
    self.mapChannels = channels
    print("map height: %s, width: %s, channels: %s"%(height, width, channels))

    # Binarize the Image
    self.mapImageBW = 255 * np.ones_like(self.mapImageGS, dtype=np.uint8)
    self.mapImageBW[self.mapImageGS == 0] = 0

    # Obtain the car length and width in pixels
    self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
    self.robotLength = int(car_length / self.map_info.resolution + 0.5)
    self.collision_delta = collision_delta

  # Check if the passed config is in collision
  # config: The configuration to check (in meters and radians)
  # Returns False if in collision, True if not in collision
  def get_state_validity(self, config):
    
    # Convert the configuration to map-coordinates -> mapConfig is in pixel-space
    mapConfig = Utils.world_to_map(config, self.map_info)
    # print("Origin (%s, %s) "%(mapConfig[0], mapConfig[1]))

    # bounding box, x1 is >= 0
    # values should be pixel coord (ints)
    x1 = max(int((mapConfig[0]-0.5*self.robotWidth)), 0)
    y1 = max(int((mapConfig[1]-0.5*self.robotLength)), 0)
    x2 = min(int((mapConfig[0]+0.5*self.robotWidth)), self.mapWidth-1) 
    y2 = min(int((mapConfig[1]+0.5*self.robotLength)), self.mapHeight-1)
    
    # Debug
    # print("checking bounding box from (x1:%s, y1:%s) to (x2:%s, y2:%s"%(x1,y1,x2,y2))

    # four loops along the lines
    # need to check boundary conditions
    # Assumption is the image 255=valid, 0=invalid
    for dy in range(y2-y1):
      val = self.mapImageBW[y1+dy, x1]
      if val == 255: return False
    for dx in range(x2-x1):
      val = self.mapImageBW[y2, x1+dx]
      if val == 255: return False
    for dy in range(y2-y1):
      val = self.mapImageBW[y2-dy, x2]
      if val == 255: return False
    for dx in range(x2-x1):
      val = self.mapImageBW[y1, x2-dx]
      if val == 255: return False
    """
    for dx in range(x2-x1):
      for dy in range(x2-x1):
        if self.mapImageBW[y1+dy, x1+dx]==0:
          print "invalid state found"
          return False
    """
    return True

  # Discretize the path into N configurations, where N = path_length / self.collision_delta
  #
  # input: an edge represented by the start and end configurations
  #
  # return three variables:
  # list_x - a list of x values of all intermediate points in the path
  # list_y - a list of y values of all intermediate points in the path
  # edgeLength - The euclidean distance between config1 and config2
  def discretize_edge(self, config1, config2):
    
    list_x, list_y = [], []
    edgeLength = np.sqrt(np.power(config1[0] - config2[0], 2) + np.power(config1[1] - config2[1], 2))
    N = int(math.floor(edgeLength/self.collision_delta))
    if N == 0:
      #print "Discretizing edge: (%s, %s) => (%s, %s)"%(config1[0], config1[1], config2[0], config2[1])
      #print "Discretizing edge: Found N = 0, returning (%s, %s)"%(config1[0], config1[1])
      return [config1[0]], [config1[1]], edgeLength
      
    dx=(config2[0]-config1[0])/N
    dy=(config2[1]-config1[1])/N
    for dt in range(N):
            list_x.append(config1[0] + dx*dt)
            list_y.append(config1[1] + dy*dt)
    return list_x, list_y, edgeLength


  # Check if there is an unobstructed edge between the passed configs
  # config1, config2: The configurations to check (in meters and radians)
  # Returns false if obstructed edge, True otherwise
  def get_edge_validity(self, config1, config2):
    # -----------------------------------------------------------
    # YOUR CODE HERE
    #
    # Check if endpoints are obstructed, if either is, return false
    # Find path between two configs by connecting them with a straight line
    # Discretize the path with the discretized_edge function above
    # Check if all configurations along path are obstructed
    # -----------------------------------------------------------
    lx, ly, length = self.discretize_edge(config1, config2)
    valid = True
    for x, y in zip(lx, ly):
      if not self.get_state_validity([x,y,0.0]):
        valid = False
    return valid

  # Lazy - this is basically the same as get_edge_validity, but returns the 
  #        discretized edge a little more cleanly
  def update_path(self, config1, config2):
    lx, ly, length = self.discretize_edge(config1, config2)
    valid = True
    if len(lx) < 2:
      valid = False
    else:
      for x, y in zip(lx, ly):
        if not self.get_state_validity([x,y,0.0]):
          valid = False
          break
    if valid:
      return True, lx, ly
    return False, None, None


# Write Your Test Code For Debugging
#if __name__ == '__main__':
#       return
        # Write test code here!
