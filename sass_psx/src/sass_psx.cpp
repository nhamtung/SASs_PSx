#include "sass_psx/sass_psx.h"

namespace teleop_twist_joy
{
    TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
    {
        cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        ROS_INFO("sass_psx.cpp- Publish the topic /cmd_vel");

        // enable_manual_sub = nh->subscribe<agv_define::io_digital>("/enable_manual", 1, boost::bind(&TeleopTwistJoy::enableManualCallback, this, _1));
        // ROS_INFO("sass_psx.cpp- Subscriber topic /enable_manual");
        joy_sub = nh->subscribe<sensor_msgs::Joy>("/joy", 1, boost::bind(&TeleopTwistJoy::joyCallback, this, _1));
        ROS_INFO("sass_psx.cpp- Subscriber topic /joy");

        nh_param->param<int>("enable_button", enable_button, 0);
        nh_param->param<int>("enable_turbo_button", enable_turbo_button, -1);

        if (nh_param->getParam("axis_joy_linear_left", axis_linear_map)){
            nh_param->getParam("scale_linear", scale_linear_map["normal"]);
            nh_param->getParam("scale_linear_turbo", scale_linear_map["turbo"]);
        }else{
            nh_param->param<int>("axis_joy_linear_left", axis_linear_map["x"], 1);
            nh_param->param<double>("scale_linear", scale_linear_map["normal"]["x"], 0.5);
            nh_param->param<double>("scale_linear_turbo", scale_linear_map["turbo"]["x"], 1.0);
        }

        if (nh_param->getParam("axis_joy_angular_right", axis_angular_map)){
            nh_param->getParam("scale_angular", scale_angular_map["normal"]);
            nh_param->getParam("scale_angular_turbo", scale_angular_map["turbo"]);
        }else{
            nh_param->param<int>("axis_joy_angular_right", axis_angular_map["yaw"], 0);
            nh_param->param<double>("scale_angular", scale_angular_map["normal"]["yaw"], 0.5);
            nh_param->param<double>("scale_angular_turbo", scale_angular_map["turbo"]["yaw"], scale_angular_map["normal"]["yaw"]);
        }

        ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", enable_button);
        ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo on button %i.", enable_turbo_button);

        for (std::map<std::string, int>::iterator it = axis_linear_map.begin(); it != axis_linear_map.end(); ++it){
            ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
            it->first.c_str(), it->second, scale_linear_map["normal"][it->first]);
            ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo for linear axis %s is scale %f.", it->first.c_str(), scale_linear_map["turbo"][it->first]);
        }
        for (std::map<std::string, int>::iterator it = axis_angular_map.begin(); it != axis_angular_map.end(); ++it){
            ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
            it->first.c_str(), it->second, scale_angular_map["normal"][it->first]);
            ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo for angular axis %s is scale %f.", it->first.c_str(), scale_angular_map["turbo"][it->first]);
        }
        sent_disable_msg = false;
    }
    TeleopTwistJoy::~TeleopTwistJoy(){};

    double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map, const std::map<std::string, double>& scale_map, const std::string& fieldname){
        if (axis_map.find(fieldname) == axis_map.end() || scale_map.find(fieldname) == scale_map.end() || joy_msg->axes.size() <= axis_map.at(fieldname)){
            return 0.0;
        }
        // ROS_WARN("fieldname: %s", fieldname.c_str());
        // ROS_INFO("axis_map.at(fieldname): %d", axis_map.at(fieldname));
        // ROS_INFO("joy_msg->axes[axis_map.at(fieldname)]: %f", joy_msg->axes[axis_map.at(fieldname)]);
        // ROS_INFO("scale_map.at(fieldname): %f", scale_map.at(fieldname));
        return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
    }
    // void TeleopTwistJoy::enableManualCallback(const agv_define::io_digital::ConstPtr& msg){
    //     if(msg->mode == 1){
    //         enable_manual = true;
    //         ROS_INFO("sass_psx.cpp - Enable Manual Mode");
    //     }else{
    //         enable_manual = false;
    //         ROS_INFO("sass_psx.cpp - Disable Manual Mode");
    //     }
    // }
    void TeleopTwistJoy::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map){
        ROS_INFO_THROTTLE(3, "sass_psx.cpp - sendCmdVelMsg: %s", which_map.c_str());
        geometry_msgs::Twist cmd_vel_msg;

        cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
        // cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
        // cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
        cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
        // cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
        // cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");

        cmd_vel_pub.publish(cmd_vel_msg);
        ROS_INFO("sass_psx.cpp - cmd_vel: linear: %f, angular: %f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
        sent_disable_msg = false;
    }

    void TeleopTwistJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        // ROS_INFO("enable_turbo_button: %d", enable_turbo_button);
        // ROS_INFO("joy_msg->buttons.size() vs enable_turbo_button: %d, %d", joy_msg->buttons.size(), enable_turbo_button);
        // ROS_INFO("joy_msg->buttons[enable_turbo_button]: %d", joy_msg->buttons[enable_turbo_button]);
        if(enable_manual){
            if (enable_turbo_button >= 0 && joy_msg->buttons.size() > enable_turbo_button && joy_msg->buttons[enable_turbo_button]){
                sendCmdVelMsg(joy_msg, "turbo");
            }else if (joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button]){
                sendCmdVelMsg(joy_msg, "normal");
            }else{
                if (!sent_disable_msg){
                    ROS_INFO("sass_psx.cpp - sent_disable_msg = false");
                    geometry_msgs::Twist cmd_vel_msg;
                    cmd_vel_pub.publish(cmd_vel_msg);
                    sent_disable_msg = true;
                }
            }
        }else{
            ROS_INFO_THROTTLE(2, "sass_psx.cpp - Please enable the manual mode");
        }
    }
}  // namespace teleop_twist_joy