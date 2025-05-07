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


// void setCamParam(sensor_msgs::CameraInfo &cam_info)
// {
//     cam_info.width  = c_width;
//     cam_info.height = c_height;

//     // distortion: {k1 k2 p1 p2 k3}
//     cam_info.D.resize(5);
//     cam_info.D = {c_k1, c_k2, c_p1, c_p2, 0.f};  

//     // camera matrix: {{fx,0,cx},{0,fy,cy},{0,0,1}}; 
//     cam_info.K = {c_fx,    0, c_cx,              
//                      0, c_fy, c_cy,
//                      0,    0,    1};     
    
//     // rectification: I_3                        
//     cam_info.R = {1.f, 0.f, 0.f,                 
//                   0.f, 1.f, 0.f,
//                   0.f, 0.f, 1.f};    

//     // projection: {{fx',0,cx',Tx=0},{0,fy',cy',Ty=0},{0,0,1,0}} (Tx=Ty=0 for monocular)              
//     cam_info.P = {c_fx,    0, c_cx, 0,
//                      0, c_fy, c_cy, 0,
//                      0,    0,    1, 0};          
// }
std::string getTimestampFilename() 
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm* now_tm = std::localtime(&now_time);

    std::ostringstream oss;
    oss << "log_"
        << std::put_time(now_tm, "%Y%m%d_%H%M%S")
        << ".avi";

    return oss.str();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_pub_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    image_transport::ImageTransport it(nh);

    /* PARAMS */
    bool _visualize;
    int _cam_dev_id, _c_width, _c_height, _c_fps, _logging_fps;
    // float _c_k1, _c_k2, _c_p1, _c_p2, _c_fx, _c_fy, _c_cx, _c_cy;
    std::string _cam_name, _cam_frame, _img_topic;

    nh.param<int>("c_width",   _c_width, 1280);
    nh.param<int>("c_height", _c_height, 800);
    nh.param<int>("c_fps",       _c_fps, 120);

    // nh_.param<float>("c_k1", _c_k1, 0);
    // nh_.param<float>("c_k2", _c_k2, 0);
    // nh_.param<float>("c_p1", _c_p1, 0);
    // nh_.param<float>("c_p2", _c_p2, 0);

    // nh_.param<float>("c_fx", _c_fx, 0);
    // nh_.param<float>("c_fy", _c_fy, 0);
    // nh_.param<float>("c_cx", _c_cx, 0);
    // nh_.param<float>("c_cy", _c_cy, 0);

    nh_private.param<std::string>("camera_name",   _cam_name,      "/camera");
    nh_private.param<std::string>("camera_frame", _cam_frame, "/camera_link");
    nh_private.param<std::string>("image_topic",  _img_topic,   "/image_raw");
    
    nh_private.param<int>("cam_dev_id", _cam_dev_id,     0);
    nh_private.param<bool>("visualize",  _visualize, false);
    nh_private.param<int>("logging_fps",  _logging_fps, 20);

    // ros::Publisher pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>(cam_name + "/camera_info", 1);
    image_transport::Publisher pub_img = it.advertise(_cam_name + _img_topic, 1);
    ros::Publisher pub_header_img = nh.advertise<aims_als::ImgWithHeader>("/aims/header_image", 1);
    

    /** @arg
     *  - first: choose a default camera of the system
     *  - second: use the Video4Linux2 driver (backend) within OpenCV
     */
    cv::VideoCapture vcap("/dev/usb_cam", cv::CAP_V4L2);

    vcap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    vcap.set(cv::CAP_PROP_FPS,             _c_fps);
    vcap.set(cv::CAP_PROP_FRAME_WIDTH,   _c_width);
    vcap.set(cv::CAP_PROP_FRAME_HEIGHT, _c_height);
    vcap.set(cv::CAP_PROP_BUFFERSIZE, 1); 
    ROS_WARN("Exposure should be already set in bash !!");

    auto res_w = vcap.get(cv::CAP_PROP_FRAME_WIDTH);
    auto res_h = vcap.get(cv::CAP_PROP_FRAME_HEIGHT);
    // assert((res_w*res_h) == (_c_width*_c_height));

    if (!vcap.isOpened()) {
        ROS_ERROR("Can't open the camera");
        return -1;
    }


    cv::Mat cv_img;
    ros::Rate rate(_c_fps);
    const int periodic_iter = _c_fps / _logging_fps;
    int iter = 0;

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();

        vcap >> cv_img;
        // vcap.grab();  // grab frame but do not decode previous frames (avoids buffering delay)
        // vcap.retrieve(cv_img); // retrieve the latest frame only


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
