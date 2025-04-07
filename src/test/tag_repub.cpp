#include "aims_als/test/tag_repub.hpp"


TAG_REPUB::TAG_REPUB(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private):
nh_(nh),
nh_private_(nh_private)
{
    nh_private_.param<int>("big_tag_id", _b_tag_id, 2);
    nh_private_.param<int>("small_tag_id", _s_tag_id, 4);

    sub_tag_ = nh_.subscribe("/tag_detections", 1, &TAG_REPUB::tagCb, this, ros::TransportHints().tcpNoDelay());
    pub_b_tag_ = nh_.advertise<geometry_msgs::PoseStamped>("/aims/b_tag_detections", 1);
    pub_s_tag_ = nh_.advertise<geometry_msgs::PoseStamped>("/aims/s_tag_detections", 1);

    tag_.pose_stm.header.frame_id = "camera_link";
}

void TAG_REPUB::tagCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    size_t detected_num = msg->detections.size();

    switch (detected_num)
    {
        case 2:
            for (const auto &tag : msg->detections)
            {
                tag_.id = tag.id[0];
                tag_.pose_stm.pose.position = tag.pose.pose.pose.position;
                tag_.pose_stm.pose.orientation = tag.pose.pose.pose.orientation;
                tags_.push_back(tag_);
            }

            for (auto &tag : tags_) {
                if (tag.id == _b_tag_id) {
                    tag.pose_stm.header.stamp = ros::Time::now();
                    pub_b_tag_.publish(tag.pose_stm);
                } else {
                    tag.pose_stm.header.stamp = ros::Time::now();
                    pub_s_tag_.publish(tag.pose_stm);
                }
            }
            tags_.clear();
            break;

        case 1:
            tag_.id = msg->detections[0].id[0];
            tag_.pose_stm.pose.position = msg->detections[0].pose.pose.pose.position;
            tag_.pose_stm.pose.orientation = msg->detections[0].pose.pose.pose.orientation;

            if (tag_.id == _b_tag_id) {
                tag_.pose_stm.header.stamp = ros::Time::now();
                pub_b_tag_.publish(tag_.pose_stm);
            } else {
                tag_.pose_stm.header.stamp = ros::Time::now();
                pub_s_tag_.publish(tag_.pose_stm);
            }
            break;

        default:
            ROS_WARN("The number of detected tags is neither one or two");
            // ros service to safer node
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_repub_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    std::unique_ptr<TAG_REPUB> tag_repub_obj = std::make_unique<TAG_REPUB>(nh, nh_private);
    
    tag_repub_obj->run();

    return 0;
}