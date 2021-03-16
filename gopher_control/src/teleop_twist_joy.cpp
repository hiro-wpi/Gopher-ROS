/**
Software License Agreement (BSD)
\file      teleop_node.cpp teleop_twist_joy.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
Modified version of the original teleop_node.cpp and teleop_twist_joy.cpp
To control a robot base and its active camera using game controller
*/


#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  void sendPitchYawMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher pitch_yaw_pub;

  // base
  std::map<std::string, int> axis_base_linear_map;
  std::map< std::string, std::map<std::string, double> > scale_base_linear_map;

  std::map<std::string, int> axis_base_angular_map;
  std::map< std::string, std::map<std::string, double> > scale_base_angular_map;

  // camera
  std::map<std::string, int> axis_camera_angular_map;
  std::map< std::string, std::map<std::string, double> > scale_camera_angular_map;

  int turbo_button;
  bool turbo_mode;
  bool trigger_turbo;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;
  pimpl_->turbo_mode = false;
  pimpl_->trigger_turbo = true;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->pitch_yaw_pub = nh->advertise<geometry_msgs::Twist>("pitch_yaw", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

  nh_param->param<int>("turbo_button", pimpl_->turbo_button, 0);
  
  if (nh_param->getParam("axis_base_linear_fwd", pimpl_->axis_base_linear_map) &&
      nh_param->getParam("axis_base_linear_bwd", pimpl_->axis_base_linear_map))
  {
    nh_param->getParam("scale_base_linear", pimpl_->scale_base_linear_map["normal"]);
    nh_param->getParam("scale_base_linear_turbo", pimpl_->scale_base_linear_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_base_linear_fwd", pimpl_->axis_base_linear_map["x_fwd"], 5);
    nh_param->param<int>("axis_base_linear_bwd", pimpl_->axis_base_linear_map["x_bwd"], 2);
    nh_param->param<double>("scale_base_linear", pimpl_->scale_base_linear_map["normal"]["x_fwd"], 0.5);
    nh_param->param<double>("scale_base_linear", pimpl_->scale_base_linear_map["normal"]["x_bwd"], 0.5);
    nh_param->param<double>("scale_base_linear_turbo", pimpl_->scale_base_linear_map["turbo"]["x_fwd"], 1.0);
    nh_param->param<double>("scale_base_linear_turbo", pimpl_->scale_base_linear_map["turbo"]["x_bwd"], 1.0);
  }

  if (nh_param->getParam("axis_base_angular", pimpl_->axis_base_angular_map))
  {
    nh_param->getParam("scale_base_angular", pimpl_->scale_base_angular_map["normal"]);
    nh_param->getParam("scale_base_angular_turbo", pimpl_->scale_base_angular_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_base_angular", pimpl_->axis_base_angular_map["yaw"], 0);
    nh_param->param<double>("scale_base_angular", pimpl_->scale_base_angular_map["normal"]["yaw"], 0.5);
    nh_param->param<double>("scale_base_angular_turbo",
        pimpl_->scale_base_angular_map["turbo"]["yaw"], pimpl_->scale_base_angular_map["normal"]["yaw"]);
  }

  if (nh_param->getParam("axis_camera_angular", pimpl_->axis_camera_angular_map))
  {
    nh_param->getParam("scale_camera_angular", pimpl_->scale_camera_angular_map["normal"]);
    nh_param->getParam("scale_camera_angular_turbo", pimpl_->scale_camera_angular_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_camera_angular_pitch", pimpl_->axis_camera_angular_map["pitch"], 4);
    nh_param->param<double>("scale_camera_angular", pimpl_->scale_camera_angular_map["normal"]["pitch"], 0.8);
    nh_param->param<double>("scale_camera_angular_turbo",
        pimpl_->scale_camera_angular_map["turbo"]["pitch"], pimpl_->scale_camera_angular_map["normal"]["pitch"]);
      
    nh_param->param<int>("axis_camera_angular_yaw", pimpl_->axis_camera_angular_map["yaw"], 3);
    nh_param->param<double>("scale_camera_angular", pimpl_->scale_camera_angular_map["normal"]["yaw"], 0.8);
    nh_param->param<double>("scale_camera_angular_turbo",
        pimpl_->scale_camera_angular_map["turbo"]["yaw"], pimpl_->scale_camera_angular_map["normal"]["yaw"]);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Turbo enable and disable button %i.", pimpl_->turbo_button);
  for (std::map<std::string, int>::iterator it = pimpl_->axis_base_linear_map.begin();
      it != pimpl_->axis_base_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Base linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_base_linear_map["normal"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_base_angular_map.begin();
      it != pimpl_->axis_base_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Base angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_base_angular_map["normal"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_camera_angular_map.begin();
      it != pimpl_->axis_camera_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Camera angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_camera_angular_map["normal"][it->first]);
  }

  ROS_INFO("Press RB and LB simultaneously to initialize");
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.angular.z = 0;

  double x_forward = - getVal(joy_msg, axis_base_linear_map, scale_base_linear_map[which_map], "x_fwd");
  double x_backward = - getVal(joy_msg, axis_base_linear_map, scale_base_linear_map[which_map], "x_bwd");
  if (x_forward != 0 && x_backward != 0)
  {
    cmd_vel_msg.linear.x = (x_forward - x_backward) /2;
  }
  cmd_vel_msg.angular.z = getVal(joy_msg, axis_base_angular_map, scale_base_angular_map[which_map], "yaw");
  
  cmd_vel_pub.publish(cmd_vel_msg);
}

void TeleopTwistJoy::Impl::sendPitchYawMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                           const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist pitch_yaw_msg;

  pitch_yaw_msg.angular.z = getVal(joy_msg, axis_camera_angular_map, scale_camera_angular_map[which_map], "yaw");
  pitch_yaw_msg.angular.y = getVal(joy_msg, axis_camera_angular_map, scale_camera_angular_map[which_map], "pitch");

  pitch_yaw_pub.publish(pitch_yaw_msg);
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Switch between normal and turbo (button)
  std::string turbo_mode_info;
  if (joy_msg->buttons[turbo_button] && !trigger_turbo)
  {
    turbo_mode = !turbo_mode;
    if (turbo_mode)
    {
      turbo_mode_info = "turbo";
    }
    else
    {
      turbo_mode_info = "normal";
    }
    ROS_INFO("Switch mode to %s", turbo_mode_info.c_str());

    trigger_turbo = true;
  }
  else if (!joy_msg->buttons[turbo_button] && trigger_turbo)
  {
    trigger_turbo = false;
  }
  
  // Send joy signals
  if (turbo_mode)
  {
    sendCmdVelMsg(joy_msg, "turbo");
    sendPitchYawMsg(joy_msg, "turbo");
  }
  else
  {
    sendCmdVelMsg(joy_msg, "normal");
    sendPitchYawMsg(joy_msg, "normal");
  }
}

}  // namespace teleop_twist_joy


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop_twist_joy_node");

  ros::NodeHandle nh(""), nh_param("~");
  teleop_twist_joy::TeleopTwistJoy joy_teleop(&nh, &nh_param);

  ros::spin();
}
