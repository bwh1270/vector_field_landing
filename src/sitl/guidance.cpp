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
 * @brief Guidance Vector Field Test
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 * @date 2025.02.04
 */

/* C++ Header */
#include <iostream>
#include <cassert>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>

/* ROS */
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

#include "mavros_msgs/SetMode.h"

/* Aims */
#include "aims_als/lib/math.h"
#include "aims_als/lib/util.h"
#include "aims_als/lib/uav.h"

using namespace aims_fly;


class Guidance
{
    public:
        Guidance(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~Guidance() = default;

        void run() { ros::spin(); };
    
    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_odom_uav_;
        ros::Subscriber sub_odom_ugv_;
        ros::Publisher  pub_odom_vel_;
        ros::Publisher  pub_bool_land_;
        ros::ServiceClient set_mode_client_;

        // parameters
        double _k1, _k2, _eps1, _eps2; 
        double _r_max;
        double _phi_des, _phi_1;
        double _alpha;
        double _eps_z, _impact_vel;

        struct gv_t {
            Eigen::Vector3d p;
            Eigen::Vector4d q;
            Eigen::Vector3d v;
        };
        gv_t gv_;

        // variables
        struct polar_t {
            double r;      // radius
            double phi;    // varphi
            double e_phi;  // error of varphi
            double rdot;
            double phidot;
        };
        polar_t p_;

        struct vector_field_t {
            double K1;     // k1'
            double K2;     // k2'
            double Xr;     // Xr
            double Xphi;   // Xphi
        };
        vector_field_t gvf_;

        Eigen::Vector3d v_wb_f_;
        Eigen::Vector3d Vc_;

        void convertPolarCoordinate(const Eigen::Vector3d &P, const Eigen::Vector3d &V);
        void computeVectorField();
        void computeControlLaw();
        void uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
        void ugvOdomCb(const nav_msgs::Odometry::ConstPtr &msg);

        void setMode();
};

Guidance::Guidance(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
: nh_(nh), nh_private_(nh_private)
{
    v_wb_f_ << 0., 0., 0.;

    nh_private_.param<double>("k1",     _k1, 1);
    nh_private_.param<double>("k2",     _k2, 1);
    nh_private_.param<double>("eps1", _eps1, 2);
    nh_private_.param<double>("eps2", _eps2, 2);

    nh_private_.param<double>("r_max",     _r_max, 10.0);

    nh_private_.param<double>("phi_des", _phi_des, 135);
    nh_private_.param<double>("phi_1",     _phi_1, 115);

    nh_private_.param<double>("alpha", _alpha, 3.14);

    nh_private_.param<double>("eps_z",           _eps_z, 0.3);
    nh_private_.param<double>("impact_vel", _impact_vel, 0.3);


    _phi_des = deg2rad(_phi_des);
    _phi_1   = deg2rad(_phi_1);


    sub_odom_uav_ = nh_.subscribe("/mavros/local_position/odom", 1, &Guidance::uavOdomCb, this, ros::TransportHints().tcpNoDelay());
    sub_odom_ugv_ = nh_.subscribe("/odom", 1, &Guidance::ugvOdomCb, this, ros::TransportHints().tcpNoDelay());

    pub_odom_vel_ = nh_.advertise<nav_msgs::Odometry>("/aims/gv_esti", 1);
    // pub_odom_vel_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    pub_bool_land_ = nh_.advertise<std_msgs::Bool>("/aims/land", 1);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void Guidance::convertPolarCoordinate(const Eigen::Vector3d &P, const Eigen::Vector3d &V)
{
    p_.r   = sqrt(P(0)*P(0) + P(2)*P(2));
    p_.phi = atan2(P(2), P(0)); 
    p_.e_phi = p_.phi - _phi_des;

    p_.rdot = V(0)*cos(p_.phi) + V(2)*sin(p_.phi);
    p_.phidot = (1./p_.r) * (-V(0)*sin(p_.phi) + V(2)*cos(p_.phi));
}

void Guidance::computeVectorField()
{
    gvf_.K1 = _k1 / tanh(1. / _eps1);
    gvf_.K2 = _k2 / tanh(_phi_des / (_eps2 * abs(_phi_des - _phi_1)));
    // gvf_.K1 = _k1;
    // gvf_.K2 = _k2;

    gvf_.Xr = p_.r / (_eps1 * _r_max);
    gvf_.Xphi = p_.e_phi / (_eps2 * abs(_phi_des - _phi_1));
}

void Guidance::computeControlLaw()
{
    // Vd
    const double Vd_r = -gvf_.K1 * tanh(gvf_.Xr);
    const double Vd_phi = -gvf_.K2 * tanh(gvf_.Xphi);

    // Vdot_d
    const double Vdot_d_r = -(gvf_.K1 / (_eps1 * _r_max)) * (p_.rdot / pow(cosh(gvf_.Xr), 2)) - gvf_.K2 * tanh(gvf_.Xphi) * p_.phidot;
    const double Vdot_d_phi = -(gvf_.K2 / (_eps2 * abs(_phi_des - _phi_1))) * (p_.phidot / pow(cosh(gvf_.Xphi), 2)) - gvf_.K1 * tanh(gvf_.Xr) * p_.phidot;
    
    // Vcmd = Vd + (1/alpha)[Vdot_d]
    const double Vc_r = Vd_r + Vdot_d_r / _alpha;
    const double Vc_phi = Vd_phi + Vdot_d_phi / _alpha;

    Vc_(0) = Vc_r * cos(p_.phi) - Vc_phi * sin(p_.phi);
    Vc_(1) = 0.;
    Vc_(2) = Vc_r * sin(p_.phi) + Vc_phi * cos(p_.phi);
}

void Guidance::uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    const Eigen::Vector3d p = point2vec(msg->pose.pose.position);
    const Eigen::Vector4d q = quat2vec(msg->pose.pose.orientation);
    const Eigen::Vector3d v = vec2vec(msg->twist.twist.linear);
    const Eigen::Matrix3d R = q2R(q); 
    const Eigen::Vector3d V_wb = R*v;
    v_wb_f_ = lpfv3(V_wb, 0.3, v_wb_f_); 

    convertPolarCoordinate(p-gv_.p, v_wb_f_ - gv_.v);
    computeVectorField();
    computeControlLaw();

    // ROS msg
    nav_msgs::Odometry cmd;
    cmd.pose.pose.position.x = gv_.p(0);
    cmd.pose.pose.position.y = gv_.p(1);
    cmd.pose.pose.position.z = gv_.p(2);
    cmd.pose.pose.orientation.w = gv_.q(0);
    cmd.pose.pose.orientation.x = gv_.q(1);
    cmd.pose.pose.orientation.y = gv_.q(2);
    cmd.pose.pose.orientation.z = gv_.q(3);
    cmd.twist.twist.linear.x = Vc_(0) + gv_.v(0);
    cmd.twist.twist.linear.y = Vc_(1);
    cmd.twist.twist.linear.z = Vc_(2);

    // geometry_msgs::Twist cmd;
    // cmd.linear.x = Vc_(0) + gv_.v(0);
    // cmd.linear.y = Vc_(1);
    // cmd.linear.z = Vc_(2);

    if (p(2) <= gv_.p(2) + _eps_z) {
        std_msgs::Bool land_msg;
        land_msg.data = true;
        pub_bool_land_.publish(land_msg);
        // cmd.linear.x = 0.;
        // cmd.linear.y = 0.;
        // // cmd.twist.twist.linear.z = -_impact_vel;
        // cmd.linear.z = -_impact_vel;
        // pub_odom_vel_.publish(cmd);
        setMode();
        while (ros::ok()) {
            
        }
        return;
    }

    pub_odom_vel_.publish(cmd);
}

void Guidance::setMode()
{
    // Wait for the service to become available
    if (!set_mode_client_.waitForExistence(ros::Duration(0.1))) {
        ROS_ERROR("SetMode service not available!");
        return;
    }

    // Create a service request
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 92;  // MAV_MODE_AUTO_DISARMED
    srv.request.custom_mode = "";  // Leave blank to use base_mode

    // Call the service
    if (set_mode_client_.call(srv)) {
        if (srv.response.mode_sent) {
            ROS_INFO("Successfully changed mode to AUTO DISARMED");
        } else {
            ROS_WARN("Mode change request sent, but not confirmed.");
        }
    } else {
        ROS_ERROR("Failed to call set_mode service");
    }
}

void Guidance::ugvOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
    gv_.p = point2vec(msg->pose.pose.position);
    gv_.p(2) = 0.45;
    gv_.q = quat2vec(msg->pose.pose.orientation);
    gv_.v = vec2vec(msg->twist.twist.linear);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guidance_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    auto guidance_obj = std::make_unique<Guidance>(nh, nh_private);
    
    guidance_obj->run();

    return 0;
}