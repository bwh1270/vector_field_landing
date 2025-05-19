/**
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 */
#include <iostream>
#include <cassert>
#include <memory>
#include <filesystem> // C++17 filesystem API

/* For OpenCV lib */
#include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

/* For being bridge between ROS Image and OpenCV */
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CameraInfo.h"

/* For ROS image data */
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "aims_als/ImgWithHeader.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "siyi_img_pub_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    image_transport::ImageTransport it(nh);

    /* PARAMS */
    bool _visualize;
    int _c_width, _c_height, _c_fps, _logging_fps;
    std::string _cam_name, _cam_frame, _img_topic, _cam_dev;

    nh.param<int>("c_width",   _c_width, 1280);
    nh.param<int>("c_height", _c_height, 800);
    nh.param<int>("c_fps",       _c_fps, 120);

    nh_private.param<std::string>("camera_name",   _cam_name,      "/camera");
    nh_private.param<std::string>("camera_frame", _cam_frame, "/camera_link");
    nh_private.param<std::string>("image_topic",  _img_topic,   "/image_raw");

    nh_private.param<bool>("visualize",  _visualize, false);
    nh_private.param<int>("logging_fps",  _logging_fps, 20);

    image_transport::Publisher pub_img = it.advertise(_cam_name + _img_topic, 1);
    ros::Publisher pub_header_img = nh.advertise<aims_als::ImgWithHeader>("/aims/header_image", 1);
    

    /** @arg
     *  - first: choose a default camera of the system
     *  - second: use the Video4Linux2 driver (backend) within OpenCV
     */
    std::string pipeline = "rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! decodebin ! videoconvert ! appsink";
    cv::VideoCapture siyi{pipeline, cv::CAP_GSTREAMER};


    auto res_w = siyi.get(cv::CAP_PROP_FRAME_WIDTH);
    auto res_h = siyi.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "[W,H] = [" << res_w << ", " << res_h << "]" << std::endl;

    if (!siyi.isOpened()) {
        ROS_ERROR("Can't open the camera");
        return -1;
    }


    cv::Mat cv_img, cv_img_tmp;
    ros::Rate rate(_c_fps);
    const int periodic_iter = _c_fps / _logging_fps;
    int iter = 0;

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();

        siyi >> cv_img_tmp;
        // vcap.grab();  // grab frame but do not decode previous frames (avoids buffering delay)
        // vcap.retrieve(cv_img); // retrieve the latest frame only
        cv::flip(cv_img_tmp, cv_img, 0);  // Flip vertically

        if (cv_img.empty()) {
            ROS_WARN("Empty image");
            continue;
        }
        
        // To publish image data with ROS message, 
        // use cv_bridge to convert OpenCV data structure to ROS message

        sensor_msgs::ImagePtr img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();

        aims_als::ImgWithHeader header_img_msg;
        header_img_msg.header.stamp = now;
        header_img_msg.header.frame_id = _cam_frame;
        header_img_msg.image = *img_ptr;      

        pub_header_img.publish(header_img_msg);

        if (iter == periodic_iter) {
            pub_img.publish(img_ptr);
            iter = 0;
        }

        if (_visualize) {
            // Calculate the midpoints
            int mid_x = _c_width / 2;
            int mid_y = _c_height / 2;

            // Draw a vertical line at the midpoint
            cv::line(cv_img, cv::Point(mid_x, 0), cv::Point(mid_x, _c_height), cv::Scalar(0, 255, 0), 1);

            // Draw a horizontal line at the midpoint
            cv::line(cv_img, cv::Point(0, mid_y), cv::Point(_c_width, mid_y), cv::Scalar(0, 255, 0), 1);

            cv::imshow("Image with Lines", cv_img);
            cv::waitKey(1);  

            ROS_INFO("Input Image: (dimension, width, height)=(%d,%d,%d)", cv_img.dims, cv_img.rows, cv_img.cols);
        }

        ros::spinOnce();
        rate.sleep();
        ++iter;
    }

    return 0;
}
