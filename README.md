## to run data collection so far 

$ roslaunch flightgoggles nimbus.launch

$ python republish_colored_gates.py

$ python data_collection_regime.py

Then press a button on your joystick, that will start recording


## Notes
https://github.com/mit-fast/FlightGoggles

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
    - Includes initial pose and vehicle dynamics
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

### On Run Time
- In order to run the challenge, need to run the reporter node (reporter.py) and we need to load gate_locations.yaml and challenge_{difficulty}.yaml. We should be able to run this with the external renderer and level args set. When training, we SHOULD set ignore_collisions to false since the results.yaml will reflect a collision, and a bad score.

- The results.yaml file is saved in FlightGoggles/flightgoggles/flightgoggles_reporter/src. The output will say {Result: Collision} if a collision occurs (even when we set ignore collisions to true).

- After successfully navigating through a gate without collisions and completing a challenge, the results.yaml file will appear similar to below. This is a completion of the easy challenge.

        Gate0:
            Location: [1.3420574159063834, 33.600399339230584, 2.370399369341433]
            Name: Gate2
            Success: 'True'
            Time: 11.6666676
        Result: Challenge Completed

- For the IR markers, we only see the markers when they are in the image output, meaning in terms of **IMAGE PIXEL VALUES** and not phyiscal location. If we echo /uav/camera/left/ir_beacons we can see what corners are detected. If nothing is in view, we will see an empty message. If we do see something, the marker has the form:

        landmarkID: 
            data: "Gate5"
        markerID: 
            data: "3"
        x: 837.154907227
        y: 407.483398438
        z: 0.0

    Every marker in view will be saved in that one message. Notice we see the Gate the marker belongs to as well as the corner ID number, where 1 is the top left corner **of the phyiscal gate frame** and not necessarily the view we see (for example approaching from *behind* the gate). Notice z is **always** zero.

- the /control_nodes/keyboard/keyup and keydown can both be used to determine what inputs we are using to control the drone. It will register **any** keypress, not just ones that actually do anything for the drone. We can use this for recording our input data for imitiation learning if we record our flights for data colelction. I'm not entirely sure if this is the same for joystick but I assume it's very similar.

- Once a collision occurs, /uav/collision will begin publishing empty messages. Before the collision the topic is not updated.

- Echoing /sensors/imu we can see the linear acceleration and angular velocities. The controls for the drone change the angular velocities from -1 to 1 for x, y, and z. The accelerations will be less useful to use. The covariance do not change, looks like they are hard coded.

- The /diagnostics topic provides some information about the joystick / controller
