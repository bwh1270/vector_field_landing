#ifndef __TAG_REPUB__
#define __TAG_REPUB__

#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <aims_als/lib/util.h>

using namespace aims_fly;


struct tag_t
{
    int id;
    geometry_msgs::PoseStamped pose_stm;
};

class TAG_REPUB
{
    public:
        TAG_REPUB(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~TAG_REPUB() {};

        void run() { ros::spin(); };

    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber sub_tag_;
        ros::Publisher  pub_b_tag_;
        ros::Publisher  pub_s_tag_;

        tag_t tag_;
        std::vector<tag_t> tags_;
        int _b_tag_id, _s_tag_id;


        void tagCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

};

#endif