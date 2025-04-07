#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

bool constant_height = true;
nav_msgs::Odometry gv_esti;

void bTagPoseCb(const geometry_msgs::Pose::ConstPtr &msg)
{
    gv_esti.pose.pose.position.x = msg->position.x;
    gv_esti.pose.pose.position.y = msg->position.y;
    
    if (constant_height) {
        gv_esti.pose.pose.position.z = msg->position.z+3.0;
    } else {
        gv_esti.pose.pose.position.z = msg->position.z+1.3;
    }

    gv_esti.pose.pose.orientation.w = msg->orientation.w;
    gv_esti.pose.pose.orientation.x = msg->orientation.x;
    gv_esti.pose.pose.orientation.y = msg->orientation.y;
    gv_esti.pose.pose.orientation.z = msg->orientation.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tmp_esti_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    nh_private.param<bool>("constant_height", constant_height, true);

    ros::Subscriber sub_pose_b_tag_tf = nh.subscribe("/aims/b_tag_tf", 1, bTagPoseCb, ros::TransportHints().tcpNoDelay());
    ros::Publisher  pub_odom_b_tag_esti = nh.advertise<nav_msgs::Odometry>("/aims/gv_esti", 1);

    ros::Rate rate(50.);
    while (ros::ok())
    {
        pub_odom_b_tag_esti.publish(gv_esti);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
