# Evaluation

## Setup
* Merge the contents of `config` into `flightgoggles/config/` folder in the cloned `FlightGoggles` repository inside your catkin workspace.
* Merge the contents of `launch` into `flightgoggles/launch/` 
* Edit `flightgoggles/launch/scorer.launch` to include your ROS nodes required to complete the challenges.

## Running the scorer

* Make `scorer.sh` executable using `chmod +x scorer.sh`
* Run `./scorer.sh`. This will run the new launch file 25 times with the perturbed gates in `flightgoggles/config/gate_locations_*.yaml` and accumulate the results from the reporter in a `results` folder. At the end of the evaluation `scorer.py` is run which generates a `scores.yaml` which contains the individual scores for each run. 

**Note:** The perturbed gate locations are generated using the maximum allowed perturbation defined in the `FlightGoggles` repository in the `flightgoggles/config/challenges/gate_locations.yaml`. The perturbation defined in that file is the maximum absolute value perturbation in `x y yaw`. 

## Scoring metric
The formula for generating the individual score for every evaluation is `N.R-T` where `N` is the number of successful gate fly-throughs, `R` is the reward for every succesful fly through and `T` is the total time required to complete the challenge.

## Allowed topics and params

Below is the list of topics and parameters that contestants are allowed to listen to, but not change. 

```
Topics:
/bounding_box_camera/*
/clock
/control_nodes/joy
/control_nodes/keyboard/keydown
/control_nodes/keyboard/keyup
/control_nodes/universal_teleop/controls
/control_nodes/universal_teleop/events
/tf
/tf_static
/uav/camera/debug/fps
/uav/camera/left/*
/uav/collision
/uav/input/land
/uav/input/rateThrust
/uav/sensors/imu

Params:
/bounding_box_camera/*
/control_nodes/universal_teleop/joy_axes/pitch
/control_nodes/universal_teleop/joy_axes/roll
/control_nodes/universal_teleop/joy_axes/vertical
/control_nodes/universal_teleop/joy_axes/yaw
/run_id
/uav/camera/left/*
/uav/flightgoggles_imu/accelerometer_variance
/uav/flightgoggles_imu/gyroscope_variance
/uav/flightgoggles_lpf/gain_p
/uav/flightgoggles_lpf/gain_q
/uav/flightgoggles_pid/gain_d_pitch
/uav/flightgoggles_pid/gain_d_roll
/uav/flightgoggles_pid/gain_d_yaw
/uav/flightgoggles_pid/gain_i_pitch
/uav/flightgoggles_pid/gain_i_roll
/uav/flightgoggles_pid/gain_i_yaw
/uav/flightgoggles_pid/gain_p_pitch
/uav/flightgoggles_pid/gain_p_roll
/uav/flightgoggles_pid/gain_p_yaw
/uav/flightgoggles_pid/int_bound_pitch
/uav/flightgoggles_pid/int_bound_roll
/uav/flightgoggles_pid/int_bound_yaw
/uav/flightgoggles_uav_dynamics/angular_process_noise
/uav/flightgoggles_uav_dynamics/clockscale
/uav/flightgoggles_uav_dynamics/drag_coefficient
/uav/flightgoggles_uav_dynamics/ignore_collisions
/uav/flightgoggles_uav_dynamics/init_pose
/uav/flightgoggles_uav_dynamics/linear_process_noise
/uav/flightgoggles_uav_dynamics/max_prop_speed
/uav/flightgoggles_uav_dynamics/moment_arm
/uav/flightgoggles_uav_dynamics/motor_time_constant
/uav/flightgoggles_uav_dynamics/thrust_coefficient
/uav/flightgoggles_uav_dynamics/torque_coefficient
/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx
/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy
/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz
/uav/flightgoggles_uav_dynamics/vehicle_mass
/uav/gate_names
/uav/timeout
/use_sim_time
```

**Note:** `TF` provides ground truth pose which should only be used for debugging and not in the submission. In the final submission, `TF` queries involving `world`, `world_ned`, are not allowed.

