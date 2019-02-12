## Notes

On modifications:
- flightgoggles_uav_dynamics/src/flightgoggles_uav_dynamics_node.cpp change Uav_Dynamics::collisionCallback collision check from true to false.
- 


On launch files:
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
    
