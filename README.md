## Notes

On modifications:
- flightgoggles_uav_dynamics/src/flightgoggles_uav_dynamics_node.cpp change Uav_Dynamics::collisionCallback collision check from true to false.


### On launch files:
- teleopExample.launch
    - arguments: external renderer, ignore collisions, and stereo
    - the teleop config file is in universal_teleop/launch/example_input_map.yml
    - remaps outputs to uav/input topics
    - launches core.launch flightgoggles node
    - use_sim_time=true

- reporter.launch
    - arguments: external renderer, ignore collisions, stereo, and level
    - launch teleop node with example_input_map.yml
    - remaps outputs to uav/input topics
    - launches core.launch flightgoggles node
    - run reporter.py
    - use_sim_time=true
    - select level and load gate yml

- core.launch
    - arguments: use_dynamics, external renderer, ignore collisions, stereo, and run challange
    - load drone.yaml
    - load UAV dynamics, renderer, ros bridge, IR marker visualizer, rviz, stereo transform, mono transform, ENU->NED coordinate transform
    

### On config files
- drone.yaml
    - This file likely cannot be changed
    - Includes initially pose and vehicle dynamics
    - Also includes control parameters (PID, IMU variance, LPF gains)

- challenges
    - Easy, Medium, and Hard
    - These are the 'test' challenges that we can play with before they release the real challenge config on the 15th.
    - Includes updated initial pose, gate goals, 'widths'?, and the results.yaml file
    