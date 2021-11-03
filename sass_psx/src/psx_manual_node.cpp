#include "sass_psx/sass_psx.h"


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "psx_manual_node");

  ros::NodeHandle nh(""), nh_param("~");

  teleop_twist_joy::TeleopTwistJoy *joy_;
  joy_ = new teleop_twist_joy::TeleopTwistJoy(&nh, &nh_param);

  ros::spin();
  delete(joy_);
  return 0;
}