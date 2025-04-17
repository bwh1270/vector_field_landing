#ifndef __TAG_ESTI__
#define __TAG_ESTI__

#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include "aims_als/lib/util.h"
#include "aims_als/lib/math.h"
#include "aims_als/lib/lkf.h"
#include "aims_als/lib/filter.h"

using namespace aims_fly;

// using lkf_t = LKF<6,1,2>;    // constant acceleration w/o velocity measurement
// using lkf_t = LKF<6,1,4>;    // constant acceleration w/ velocity measurement
using lkf_t = LKF<4,1,2>;    // constant velocity w/o velocity measurement
// using lkf_t = LKF<4,1,4>;    // constant velocity w/ velocity measurement

using x_t = typename lkf_t::x_t;
using u_t = typename lkf_t::u_t;
using z_t = typename lkf_t::z_t;
using F_t = typename lkf_t::F_t;
using G_t = typename lkf_t::G_t;
using P_t = typename lkf_t::P_t;
using Q_t = typename lkf_t::Q_t;
using R_t = typename lkf_t::R_t;
using H_t = typename lkf_t::H_t;
using K_t = typename lkf_t::K_t;
using esti_t = typename lkf_t::esti_t;


class EstimationTag
{
    public:
        EstimationTag(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, const std::string &topicname);
        ~EstimationTag() = default;

        void run() { ros::spin(); };

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_pose_tag_tf_;
        ros::Publisher  pub_odom_tag_esti_;

        std::string topicname_;
        bool _delay_compensation;
        int _delay_comp_steps;
        bool _z_sg_filter;
        bool _xy_sg_filter;
        
        double _dt;
        double _P0;
        double _acc_cov;
        double _meas_cov;
        double _lambda;

        lkf_t lkf_;
        esti_t x_hat_;
        u_t u_;
        z_t z_;

        struct gv_t {
            double z_esti;  // Since ground vehicle is static model, esti = predict
            double z_meas;
            double alpha;
            Eigen::Vector4d q;
        };
        gv_t gv_;

        double t_prev_;
        std_msgs::Header header_;

        struct gv_sgf_t {
            int window;
            int poly;
            int deriv;
            std::vector<double> coeffs;
            std::deque<double> z;
            std::deque<double> x;
            std::deque<double> y;
            double vz;
            double vx;
            double vy;
        };
        gv_sgf_t gv_sgf_;

        void init();
        Q_t setQd();                              // discrete process noise matrix
        Q_t setQc() { return Q_t::Identity(); };  // continuous process noise matrix
        void publish(const esti_t &x_hat);
        // void measureCb(const nav_msgs::Odometry::ConstPtr &msg);
        void measureCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        
};

#endif