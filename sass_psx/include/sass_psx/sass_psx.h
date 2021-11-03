
#ifndef SASS_PSX_H
#define SASS_PSX_H
#pragma once

#include <map>
#include <string>
#include "ros/ros.h"
// #include "agv_define/io_digital.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

namespace teleop_twist_joy
{
  class TeleopTwistJoy
  {
    private:
      ros::Subscriber enable_manual_sub, joy_sub;
      ros::Publisher cmd_vel_pub;

      bool enable_manual = true;
      int enable_button;
      int enable_turbo_button;
      bool sent_disable_msg;

      std::map<std::string, int> axis_linear_map;
      std::map< std::string, std::map<std::string, double> > scale_linear_map;

      std::map<std::string, int> axis_angular_map;
      std::map< std::string, std::map<std::string, double> > scale_angular_map;

      void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
      // void enableManualCallback(const agv_define::io_digital::ConstPtr& msg);
      void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
    public:
      TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);
      ~TeleopTwistJoy();
  };
}  // namespace teleop_twist_joy
#endif
