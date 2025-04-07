#ifndef __TAG_TF__
#define __TAG_TF__

#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

#include <aims_als/lib/util.h>
#include <aims_als/lib/math.h>
#include <aims_als/lib/filter.h>

using namespace aims_fly;

class TransformationTag
{    
    public:
        TransformationTag(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~TransformationTag() = default;

        void run() { 
            ros::AsyncSpinner spinner(2);  // Use 2 threads for better callback efficiency
            spinner.start();
            ros::waitForShutdown();
        };
    
    protected:    
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_pose_b_tag_;
        ros::Subscriber sub_pose_s_tag_;
        ros::Subscriber sub_odom_uav_;
        ros::Publisher  pub_odom_b_tag_;
        ros::Publisher  pub_odom_s_tag_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        tf2_ros::TransformBroadcaster map2uav_br_, map2cam_br_;

        std::string world_frame_, uav_frame_, cam_frame_;
		Eigen::Matrix3d Rw_c_;
		Eigen::Matrix4d Tw_c_;

        struct prev_t 
        {
            bool first_iter;
            double t;
            double x;
            double y;
        };

        double _dt, _alpha;
        prev_t b_tag_prev_f_, s_tag_prev_f_;
        double head_prev_f_;

        void btagPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void stagPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void uavOdomCb(const nav_msgs::Odometry::ConstPtr &msg);

};


#endif
