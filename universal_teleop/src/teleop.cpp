#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <mav_msgs/RateThrust.h>
#include "universal_teleop/Control.h"
#include "teleop.h"

using namespace std;
namespace teleop = universal_teleop;

// OVERRIDES:

// Assumes 1KG drone.
const double idleThrust = 1.0f*9.81f;

teleop::Teleop::Teleop(void) : n(), n_private("universal_teleop"), key_override_enabled(false), joy_override_enabled(false)
{

  /* load mappings */
  if (n_private.hasParam("joy_axes")){
    n_private.getParam("joy_axes", joy_axes);
  }
  else joy_axes = { {"pitch", 1}, {"roll", 0}, {"yaw", 3}, {"vertical",2} };
  for (auto& j : joy_axes) joy_axis_map[j.second] = j.first;

  map<string, int> joy_buttons;
  if (n_private.hasParam("joy_buttons")) n_private.getParam("joy_buttons", joy_buttons);
  else joy_buttons = { {"override", 6}, {"start", 2}, {"stop", 1}, {"takeoff", 10}, {"land", 11} };
  for (auto& j : joy_buttons) joy_button_map[j.second] = j.first;

  joy_deadzones = { { "pitch", 0.000001f }, { "roll", 0.000001f }, { "yaw", 0.000001f }, { "vertical", 0.000001f } };
  n.param("universal_teleop/joy_deadzones", joy_deadzones, joy_deadzones);
//  for (auto& k: joy_deadzones) cout << k.first << " " << k.second << endl;

  std::map<std::string, int> keys;
  if (n_private.hasParam("keys")) n_private.getParam("keys", keys);
  else keys = { {"override", ' '},
                {"start", 'z'}, {"stop", 'x'},
                {"takeoff", 't'}, {"land", 'y'} };
  for (auto& k : keys) key_map[k.second] = k.first;

  std::map<std::string, int> key_axes;
  if (n_private.hasParam("key_axes")) n_private.getParam("key_axes", key_axes);
  else key_axes = { { "pitch+", 'i' }, { "pitch-", 'k' },
                    { "roll+", 'l' }, { "roll-", 'j' },
                    { "yaw+", 'a' }, { "yaw-", 'd' },
                    { "vertical+", 'w' }, { "vertical-", 's' }};
  for (auto& k : key_axes) key_axes_map[k.second] = k.first;
  key_axes_state = { { "pitch", 0 },{ "roll", 0 }, { "yaw", 0 }, { "vertical", 0 } };

  axis_scales = { { "pitch", 1.0f }, { "roll", 1.0f }, { "yaw", 1.0f }, { "vertical", 2.0f*idleThrust } };
  n.param("scales", axis_scales, axis_scales);
  //for (auto& k: axis_scales) cout << k.first << " " << k.second << endl;

  n.param("universal_teleop/send_velocity", send_velocity, true);

  /* subscribe to input sources */
  joy_sub = n.subscribe("joy", 1, &Teleop::joystick_event, this);
  keyup_sub = n.subscribe("keyboard/keyup", 1, &Teleop::keyboard_up_event, this);
  keydown_sub = n.subscribe("keyboard/keydown", 1, &Teleop::keyboard_down_event, this);

  /* publish events and control commands */  
  pub_vel = n.advertise<mav_msgs::RateThrust>("output/rateThrust", 1);
  
  pub_event = n.advertise<teleop::Event>("universal_teleop/events", 5);
  pub_control = n.advertise<teleop::Control>("universal_teleop/controls", 1);

  /* special events for UAV commands */
  pub_takeoff = n.advertise<std_msgs::Empty>("output/takeoff", 5);
  pub_land = n.advertise<std_msgs::Empty>("output/land", 5);
  pub_emergency = n.advertise<std_msgs::Empty>("output/reset", 5);
}

void teleop::Teleop::process_event(const teleop::Event& e)
{
  ROS_DEBUG_STREAM("event: " << e.event << " state: " << e.state);
//  if (e.event == "override") {
//    if (e.state == 0) {
//      // when releasing override, stop robot
//      geometry_msgs::Twist vel;
//      vel.linear.x = vel.linear.y = vel.linear.z = 0;
//      vel.angular.x = vel.angular.y = 1; // non-zero to avoid hovering when zero-ing controls
//      vel.angular.z = 0;
//      pub_vel.publish(vel);
//    }
//  }
//  else {
    if ((joy_override_enabled || key_override_enabled) && e.state) {
      if (e.event == "takeoff") pub_takeoff.publish(std_msgs::Empty());
      else if (e.event == "land") pub_land.publish(std_msgs::Empty());
      else if (e.event == "emergency") pub_emergency.publish(std_msgs::Empty());
    }
//  }
  pub_event.publish(e);
}

void teleop::Teleop::joystick_event(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (last_joy_msg.axes.empty()) {
    last_joy_msg = *joy;
    for (auto& b : last_joy_msg.buttons) b = 0;
  }

  // process buttons
  for (uint32_t b = 0; b < joy->buttons.size(); b++) {
    if (joy->buttons[b] != last_joy_msg.buttons[b]) {
      teleop::Event e;
      if (joy_button_map.find(b) == joy_button_map.end()) e.event = "unknown";
      else e.event = joy_button_map[b];
      e.state = joy->buttons[b];

      if (e.event == "override") joy_override_enabled = e.state;
      process_event(e);
    }
  }

  // process axis
  for (uint32_t a = 0; a < joy->axes.size(); a++) {
    if (joy->axes[a] != last_joy_msg.axes[a]) {
      teleop::Control c;
      if (joy_axis_map.find(a) == joy_axis_map.end()) c.control = "unknown";
      else c.control = joy_axis_map[a];

      if (c.control != "unknown" && std::abs(joy->axes[a]) < joy_deadzones[joy_axis_map[a]])
        c.value = 0;
      else {
        c.value = joy->axes[a];
      }

      pub_control.publish(c);
    }
  }
  
  last_joy_msg = *joy;
}

void teleop::Teleop::keyboard_up_event(const keyboard::Key::ConstPtr& key)
{
  ROS_DEBUG_STREAM("keyup: " << key->code);
  if (key_axes_map.find(key->code) != key_axes_map.end()) {
    std::string& cmd = key_axes_map[key->code];
    // Do not zero out thrust when releasing keys.
    if (cmd.substr(0, cmd.size() - 1) != "vertical") {
        key_axes_state[cmd.substr(0, cmd.size() - 1)] = 0;
    }
  }
  else {
    teleop::Event e;
    if (key_map.find(key->code) == key_map.end()) e.event = "unknown";
    else e.event = key_map[key->code];
    e.state = 0;

    if (e.event == "override"){
        key_override_enabled = false;
        // Zero out RPY rate commands. Keep thrust constant
        for (auto &axes : key_axes_state){
//          if (axes.first != "vertical"){
            axes.second = 0;
//          }
        }
    }
    process_event(e);
  }
}

void teleop::Teleop::keyboard_down_event(const keyboard::Key::ConstPtr& key)
{
  ROS_DEBUG_STREAM("keydown: " << key->code);

  if (key_axes_map.find(key->code) != key_axes_map.end()) {
    std::string& cmd = key_axes_map[key->code];
    if (cmd.substr(0, cmd.size() - 1) != "vertical"){
        key_axes_state[cmd.substr(0, cmd.size() - 1)] = (cmd[cmd.size() - 1] == '+' ? 1 : -1);
    } else {
        // Thrust should be handled differently. Should be additive for each
        // press of keyboard.
        key_axes_state["vertical"] += (cmd[cmd.size() - 1] == '+' ? 0.05f : -0.05f);
//        key_axes_state["vertical"] = max(key_axes_state["vertical"], 0);
    }
  }
  else {
    teleop::Event e;
    if (key_map.find(key->code) == key_map.end()) e.event = "unknown";
    else e.event = key_map[key->code];
    e.state = 1;

    if (e.event == "override") key_override_enabled = true;
    process_event(e);
  }
}

void teleop::Teleop::control(void)
{
    mav_msgs::RateThrust msg;
    msg.header.frame_id = "uav/imu";
    msg.header.stamp = ros::Time::now();

  // Default thrust command to idle thrust"
  msg.thrust.z = (double) idleThrust;

  if (joy_override_enabled) {
    if (last_joy_msg.axes.empty()) return;

    float pitch = last_joy_msg.axes[joy_axes["pitch"]];
    float roll = -last_joy_msg.axes[joy_axes["roll"]];
    float yaw = last_joy_msg.axes[joy_axes["yaw"]];
    float vertical = last_joy_msg.axes[joy_axes["vertical"]];

    /* check deadzones */
//    if (std::abs(pitch) < joy_deadzones["pitch"]) pitch = 0;
//    if (std::abs(yaw) < joy_deadzones["yaw"]) yaw = 0;
//    if (std::abs(roll) < joy_deadzones["roll"]) roll = 0;
//    if (std::abs(vertical) < joy_deadzones["vertical"]) vertical = 0;

    msg.angular_rates.y = std::pow(pitch,3.0f) * axis_scales["pitch"];
    msg.angular_rates.x = std::pow(roll, 3.0f) * axis_scales["roll"];
    msg.angular_rates.z = std::pow(yaw,3.0f) * axis_scales["yaw"];
    msg.thrust.z = std::pow(vertical, 3.0f) * axis_scales["vertical"] + idleThrust;

  }
  else if (key_override_enabled) {
    msg.angular_rates.y = key_axes_state["pitch"] * axis_scales["pitch"];
    msg.angular_rates.x = key_axes_state["roll"] * axis_scales["roll"];
    msg.angular_rates.z = key_axes_state["yaw"] * axis_scales["yaw"];
    msg.thrust.z = key_axes_state["vertical"] * axis_scales["vertical"] + idleThrust;
  }
    // Publish message.
    // Might be an empty message if there is no override enabled.
    pub_vel.publish(msg);
}


