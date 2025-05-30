/*********************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Woohyun Byun.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ********************************************************************************/
/** 
 * @brief Steep Angle Experiments
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 * @date 2025.03.02
 */

/* C++ Header */
#include <iostream>
#include <cassert>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>

/* ROS */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "vector_field_landing/Marker.h"
#include "vector_field_landing/TFMarker.h"

#include "mavros_msgs/SetMode.h"

/* Aims */
#include "vector_field_landing/lib/math.h"
#include "vector_field_landing/lib/util.h"
#include "vector_field_landing/lib/uav.h"

using namespace aims_fly;


class SteepAngleExp
{
    public:
        SteepAngleExp(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~SteepAngleExp() = default;

        void run() { ros::spin(); };
    
    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_marker_;
        ros::Subscriber sub_tf_marker_;
        ros::Subscriber sub_odom_uav_;
        ros::Publisher  pub_odom_cmd_;
        ros::Publisher  pub_led_color_cmd_;
        ros::Publisher  pub_data_;
        ros::ServiceClient set_mode_client_;

        const std::string hold_mode_;
        const std::string green_;
        const std::string blue_;
        const std::string red_;

        bool detected_, arrived_;

        int _max_num_setpoint;
        double _range, _T;
        double _eps_e;
        
        int num_setpoint_;
        bool first_loop_;
        double t0_;

        Eigen::Vector3d p_des_;
        Eigen::Vector3d p_uav_;

        std_msgs::String led_color_cmd_;
        nav_msgs::Odometry cmd_;

        void changeMode(const std::string &mode);
        void setLEDColor(const std::string &color);
        void setCmd(const Eigen::Vector3d &p, const Eigen::Vector4d &q);

        void uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
        void markerCb(const aims_als::Marker::ConstPtr &msg);
        void tfMarkerCb(const aims_als::TFMarker::ConstPtr &msg);

        void setSetpoint(const Eigen::Vector3d &p_marker);
        void checkArrived(const bool detected, const Eigen::Vector3d &p);

};

SteepAngleExp::SteepAngleExp(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
: nh_(nh), nh_private_(nh_private), hold_mode_("AUTO.LOITER"), green_("0,255,0"), blue_("0,0,255"), red_("255,0,0")
{
    nh_private_.param<int>("max_num_setpoint", _max_num_setpoint, 1);
    nh_private_.param<double>("range", _range, 2.0);
    nh_private_.param<double>("duration", _T, 5.0);
    nh_private_.param<double>("epsilon_error", _eps_e, 0.2);

    sub_marker_        = nh_.subscribe("/aims/fractal_detections", 1, &SteepAngleExp::markerCb, this, ros::TransportHints().tcpNoDelay());
    sub_tf_marker_     = nh_.subscribe("/aims/transformed_marker", 1, &SteepAngleExp::tfMarkerCb, this, ros::TransportHints().tcpNoDelay());
    sub_odom_uav_      = nh_.subscribe("/mavros/local_position/odom", 1, &SteepAngleExp::uavOdomCb, this, ros::TransportHints().tcpNoDelay());
    pub_odom_cmd_      = nh_.advertise<nav_msgs::Odometry>("/aims/gv_esti", 1);
    pub_led_color_cmd_ = nh_.advertise<std_msgs::String>("/aims/led_color_cmd", 1);
    pub_data_          = nh_.advertise<aims_als::Marker>("/aims/data", 1);
    set_mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    arrived_ = false;
    num_setpoint_ = 0;
    first_loop_ = true;
}

void SteepAngleExp::changeMode(const std::string &mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;

    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("Mode changed to %s", mode.c_str());
    } else {
        ROS_WARN("Failed to change mode to %s", mode.c_str());
    }
}

void SteepAngleExp::setLEDColor(const std::string &color)
{
    led_color_cmd_.data = color;
}

void SteepAngleExp::setCmd(const Eigen::Vector3d &p, const Eigen::Vector4d &q)
{
    cmd_.header.stamp = ros::Time::now();
    cmd_.pose.pose.position.x = p(0);
    cmd_.pose.pose.position.y = p(1);
    cmd_.pose.pose.position.z = p(2);
    cmd_.pose.pose.orientation.w = q(0);
    cmd_.pose.pose.orientation.x = q(1);
    cmd_.pose.pose.orientation.y = q(2);
    cmd_.pose.pose.orientation.z = q(3);
}

void SteepAngleExp::markerCb(const aims_als::Marker::ConstPtr &msg)
{
    detected_ = msg->detect;
    if ((detected_) && (!arrived_)) {
        setLEDColor(green_);
    } else {
        setLEDColor(red_);
    }
}

void SteepAngleExp::uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    p_uav_ = point2vec(msg->pose.pose.position);
}


void SteepAngleExp::setSetpoint(const Eigen::Vector3d &p_marker)
{
    const double angle = deg2rad((double) 10.* num_setpoint_); 
    p_des_(0) = p_marker(0) - _range * sin(angle); 
    p_des_(1) = p_marker(1);
    p_des_(2) = p_marker(2) + _range * cos(angle);
}

void SteepAngleExp::checkArrived(const bool detected, const Eigen::Vector3d &p)
{
    const Eigen::Vector3d err = p_des_ - p_uav_;
    const double err_size = err.norm();
    if (err_size <= _eps_e) {
        ROS_INFO("Arrived at Setpoint");
        arrived_ =  true;
        ++num_setpoint_;

        // Record the detection rate and pose accuracy
        if (first_loop_) {
            t0_ = ros::Time::now().toSec();
            first_loop_ = false;
        }
        if (abs(ros::Time::now().toSec()-t0_) <= _T) {
            aims_als::Marker data;
            data.detect = detected;
            data.pose_stmp.pose.position.x = p(0);
            data.pose_stmp.pose.position.y = p(1);
            data.pose_stmp.pose.position.z = p(2);
            pub_data_.publish(data);

            setLEDColor(blue_);
        }
        else {
            first_loop_ = true;
            arrived_ = false;
        }
    }

    if (num_setpoint_ >= _max_num_setpoint) {
        changeMode(hold_mode_);
        ROS_INFO("Terminated Successfully");
    }
}



void SteepAngleExp::tfMarkerCb(const aims_als::TFMarker::ConstPtr &msg)
{
    const bool status = msg->status;

    if (!status) {
        setLEDColor(red_);
        ROS_WARN("Gimbal Status is False");
        ROS_WARN("Check the gimbal now");
        return;
    }
    const bool detected = detected_;
    const Eigen::Vector3d p_marker = point2vec(msg->pose_stmp.pose.position);
    const Eigen::Vector4d q_des = quat2vec(msg->pose_stmp.pose.orientation);

    setSetpoint(p_marker);

    checkArrived(detected, p_marker);

    setCmd(p_des_, q_des);

    pub_odom_cmd_.publish(cmd_);
    pub_led_color_cmd_.publish(led_color_cmd_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "steep_angle_exp_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    auto steep_angle_exp_obj = std::make_unique<SteepAngleExp>(nh, nh_private);
    
    steep_angle_exp_obj->run();

    return 0;
}