## Lab 3 - Implementing PID and MPC controllers

- Arnie Larson
- 11/01/2022
- Group 11 (car 33)


### ROS fundamentals

orientation = utils.angle_to_quaternion(theta)
point = Point(x=x, y=y, z=z)
pose = Pose(point = point, orientation = orientation)

topics:
/car/car_pose => PoseStamped
/car/scan => LaserScan
/car/mux/ackermann_cmd_mux/input/navigation => AckermannDriveStamped
/laser_wanderer/rollouts => PoseArray

Pose
  point  (x, y, z)
  orientation (quaternion: x, y, z, w)
PoseStamped
  header
    seq
    stampe
    frame_id
  pose
PoseArray
  header
    seq
    stamp
    frame_id
  pose Pose[]



### PID controller

- What was my error metric?  Using the waypoint dot product with car's perpendicular orientation vector.  I liked this the most because at small lookahead, this generates a useful goal.

- My controller worked pretty well with just the P parameter.  I didn't get the ID parameters to make a large difference.  I should test with a wider look aheads and error buffers.  In general the derivative terms were somewhat small, and the integral terms were bigger 

- What is missing for implementing this on a real robot?  An ability to estimate self pose within the plan coordinate system.

- Add error plot and video


