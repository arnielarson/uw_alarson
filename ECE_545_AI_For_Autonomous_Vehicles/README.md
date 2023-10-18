# EEP_545_group1
EEP 545 Fall 2022 - Introduction To Ai For Mobile Robots

# Authors
- Arnie Larson
- Jordan Fraser
- Sanjar Normuradov


# Final Test

12/06/2022, First successsful test run completed in 1:12

### Summary of Params

#### Driver [PID Controller]

- rot_weight		0.1
- waypoint_weight	0.8  (not normalized)
- plan_lookahead	7
- speed			0.8
- ki 			0.0
- kp			0.7
- kd			0.0

#### Particle Filter

- number particles 	500
- ray_step		10
- max_range		11

#### Sensor Model

- inv_squish		0.2
- z_short		0.06
- z_max			0.02
- z_rand		0.03
- sigma_hit		4	(pixel coords)
- z_hit			0.9
- lambda		0.001

#### Motion Model  (I think a noisy model mad

- v_noise		0.025
- delta_noise		0.02
- x_fix_noise		0.025
- y_fix_noise		0.025
- theta_fix_noise	0.025


