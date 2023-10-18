## Final Project - Autonomous Robot Car Test

- Arnie Larson, Jordan Fraser, Sanjar Normuradov
- 11/28/2022
- Group 11 (car 33)


### TODO ###

- [*] Implement Way points in Planner
- [ ] Implement avoidance points in Planner+Obstacle manager
- [*] Launch controller (PID) with complex plan
- [ ] Tune controller to follow plan
- [ ] Tune plan(s) so that robot can follow 
- [ ] Generate saved test plans
- [ ] Incorporate Particle Filter with Sensor model in simulation
- [ ] Test real robot on small test plans

### Tuning Recipes ###

- Sensor Model - belief tuning parameters are in top of SensorModel.py
- Sensor Model - lidar tuning parameters are available in ParticleFilter.launch 
- Motion Model - tuning parameters are at top of MotionModel.py
- PID Controller - Model parameters are accessible in ParticleFilter.launch
- Planner - waypoints are constant arrays in Planner.py
- Planner - graph parameters (Nodes, radius, collision (step)) are in PlannerNode.launch


### Running a simple plan ###

- Add plan/simple_plan.csv to ~/.ros
- Make sure Driver is set up to load the correct plan file
- Make sure Driver.launch has the plan_topic set to the correct inferred_pose (/pf/viz/inferred_pose in ParticleFilter)
- Launch roscore, launch mushr_sim teleop, launch rviz, launch particle_filter, launch driver
- Reset robot initial pose if particle filter doesn't find the correct pose
- When ready hit enter on Driver window and go back to RVIZ to watch

### Creatign a plan ###
- Set waypoints in Planner.py
- Set filename in Planner.launch
- Plan will be saved in ~/.ros directory