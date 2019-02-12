## Notes

### On modifications:
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

### On Nodes
- reporter.py
    - pulls in challenge name, timeout (200s default), gate names, gate width, results location, and 'inflation'?
    - Subs to /uav/collision and /world , /uav/imu transforms to check for collisons and drone pose
    - Creates gate objects
    - Checks for missed gates and for completed challenge
    - Checks for interruption

- flightgoggles_marker_visualizer_node.cpp
    - This is where the *bounding box* topics come from
    - Uses custom IRMarker.msg has Gate ID, marker ID, and x,y,z location in meters
    - Subs to image_rect_color and ir_beacon and publishes two overlaid into /bounding_box_camera/RGB

- ROSClient.cpp
    - creates uav_collision, ir_beacon, and fps publishers
    - suscribes to tf messages
    - Ability to change scene (Abandoned_Factory_Morning) and (Abandoned_Factory_Sunset) and others
    - Includes camera metadata and intrinsic information
    - IR markers are the visible markers that can be seen in FlightGoggles, with z being the distance from the camera, which is NOT published (hardcoded to 0). This tells us that we can likely scan for a present marker and once we see it we can act on it. We also know the IDs of the marker so we know if this is the right gate or not.

- flightgoggles_uav_dynamic_node.cpp
    - Main UAV dynamics node. Main file interfacing drone with ros
    - Loads in params from drone.yaml
    - Default of 9.81 z-thrust comes from opposing force to gravity keeping the drone in the air (if it was less than 0, it would constantlu drift down towards the ground)
    - advertises imu, transform, and clock, subs to collision and fps
    - Big chunk of code for vehicle dynamics

- GeneralClient.cpp
    - updates and renders images, camera, and pose information

- FlightGogglesClient.cpp
    - renders environment in ros and saves images from Unity

