/**
Software License Agreement (BSD)

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

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "dynamixel_pan_tilt_msgs/PanTiltCmd.h"
#include "teleop_joy/teleop_joy.h"

#include <map>
#include <string>

namespace teleop_joy
{

    struct TeleopJoy::Impl
    {
        void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
        void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::string &which_map);
        void sendPanTiltCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::string &which_map);
        void sendPanTiltCmdPosMsg(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::string &which_map);

        ros::Subscriber joy_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher pan_tilt_cmd_vel_pub;
        ros::Publisher pan_tilt_cmd_pos_pub;
        ros::Publisher light_pub;

        int enable_button;
        int enable_turbo_button;
        int enable_button_pan_tilt_pos;

        std::map<std::string, int> axis_linear_map;
        std::map<std::string, std::map<std::string, double>> scale_linear_map;
        std::map<std::string, double> offset_linear_map;

        std::map<std::string, int> axis_angular_map;
        std::map<std::string, std::map<std::string, double>> scale_angular_map;
        std::map<std::string, double> offset_angular_map;

        bool sent_disable_msg;
    };

    /**
     * Constructs TeleopJoy.
     * \param nh NodeHandle to use for setting up the publisher and subscriber.
     * \param nh_param NodeHandle to use for searching for configuration parameters.
     */
    TeleopJoy::TeleopJoy(ros::NodeHandle *nh, ros::NodeHandle *nh_param)
    {
        pimpl_ = new Impl;

        pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        pimpl_->pan_tilt_cmd_vel_pub = nh->advertise<dynamixel_pan_tilt_msgs::PanTiltCmd>("pan_tilt/cmd_vel", 1, true);
        pimpl_->pan_tilt_cmd_pos_pub = nh->advertise<dynamixel_pan_tilt_msgs::PanTiltCmd>("pan_tilt/cmd_pos", 1, true);
        pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::Impl::joyCallback, pimpl_);

        nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
        nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);
        nh_param->param<int>("enable_button_pan_tilt_pos", pimpl_->enable_button_pan_tilt_pos, -1);

        if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
        {
            nh_param->getParam("scale_linear", pimpl_->scale_linear_map["normal"]);
            nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);
            nh_param->getParam("offset_linear", pimpl_->offset_linear_map);
        }

        if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
        {
            nh_param->getParam("scale_angular", pimpl_->scale_angular_map["normal"]);
            nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);
            nh_param->getParam("offset_angular", pimpl_->offset_angular_map);
        }

        ROS_INFO_NAMED("TeleopJoy", "Teleop enable button %i.", pimpl_->enable_button);
        ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopJoy",
                            "Turbo on button %i.", pimpl_->enable_turbo_button);
        ROS_INFO_COND_NAMED(pimpl_->enable_button_pan_tilt_pos >= 0, "TeleopJoy",
                            "Pan tilt position command enable button %i.", pimpl_->enable_button_pan_tilt_pos);
        ROS_INFO_COND_NAMED(pimpl_->enable_button_pan_tilt_pos <= 0, "TeleopJoy",
                            "Pan tilt position command enable button not set, position commands will be ignored.");
        for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
             it != pimpl_->axis_linear_map.end(); ++it)
        {
            ROS_INFO_NAMED("TeleopJoy", "Linear axis %s on %i at scale %f with offset %f.",
                           it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first], pimpl_->offset_linear_map[it->first]);
            ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopJoy",
                                "Linear axis %s turbo is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
        }
        
        for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
             it != pimpl_->axis_angular_map.end(); ++it)
        {
            ROS_INFO_NAMED("TeleopJoy", "Angular axis %s on %i at scale %f with offset %f.",
                           it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first], pimpl_->offset_angular_map[it->first]);
            ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopJoy",
                                "Angular axis %s turbo is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
        }

        pimpl_->sent_disable_msg = false;
    }

    double getVal(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_map,
                  const std::map<std::string, double> &scale_map, const std::map<std::string, double> &offset_map, const std::string &fieldname)
    {
        if (axis_map.find(fieldname) == axis_map.end() ||
            scale_map.find(fieldname) == scale_map.end() ||
            joy_msg->axes.size() <= axis_map.at(fieldname))
        {
            return 0.0;
        }

        return (joy_msg->axes[axis_map.at(fieldname)] + offset_map.at(fieldname)) * scale_map.at(fieldname);
    }

    void TeleopJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg,
                                        const std::string &which_map)
    {
        // Initializes with zeros by default.
        geometry_msgs::Twist cmd_vel_msg;

        cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "forwards") + getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "backwards");
        cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "y");
        cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "z");
        cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], offset_angular_map, "yaw");
        cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], offset_angular_map, "pitch");
        cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], offset_angular_map, "roll");

        cmd_vel_pub.publish(cmd_vel_msg);
        sent_disable_msg = false;
    }

    void TeleopJoy::Impl::sendPanTiltCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg,
                                               const std::string &which_map)
    {
        dynamixel_pan_tilt_msgs::PanTiltCmd cmd_msg;
        cmd_msg.pan_val = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "pan_vel");
        cmd_msg.tilt_val = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "tilt_vel");

        pan_tilt_cmd_vel_pub.publish(cmd_msg);
        sent_disable_msg = false;
    }

    void TeleopJoy::Impl::sendPanTiltCmdPosMsg(const sensor_msgs::Joy::ConstPtr &joy_msg,
                                               const std::string &which_map)
    {
        dynamixel_pan_tilt_msgs::PanTiltCmd cmd_msg;
        cmd_msg.pan_val = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "pan_pos");
        cmd_msg.tilt_val = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], offset_linear_map, "tilt_pos");

        pan_tilt_cmd_pos_pub.publish(cmd_msg);
        sent_disable_msg = false;
    }

    void TeleopJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
        if (enable_turbo_button >= 0 &&
            joy_msg->buttons.size() > enable_turbo_button &&
            joy_msg->buttons[enable_turbo_button])
        {
            sendCmdVelMsg(joy_msg, "turbo");
            if (joy_msg->buttons.size() > enable_button_pan_tilt_pos && joy_msg->buttons[enable_button_pan_tilt_pos])
            {
                sendPanTiltCmdPosMsg(joy_msg, "turbo");
            }
            else
            {
                sendPanTiltCmdVelMsg(joy_msg, "turbo");
            }
        }
        else if (joy_msg->buttons.size() > enable_button &&
                 joy_msg->buttons[enable_button])
        {
            sendCmdVelMsg(joy_msg, "normal");
            if (joy_msg->buttons.size() > enable_button_pan_tilt_pos && joy_msg->buttons[enable_button_pan_tilt_pos])
            {
                sendPanTiltCmdPosMsg(joy_msg, "normal");
            }
            else
            {
                sendPanTiltCmdVelMsg(joy_msg, "normal");
            }
        }
        else
        {
            // When enable button is released, immediately send a single no-motion command
            // in order to stop the robot.
            if (!sent_disable_msg)
            {
                // Initializes with zeros by default.
                geometry_msgs::Twist cmd_vel_msg;
                cmd_vel_pub.publish(cmd_vel_msg);
                sent_disable_msg = true;
            }
        }
    }

} // namespace teleop_twist_joy