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
 * @brief Detection of Fractal markers which are one of the marker types provided by the ArUco3 library.
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 * @date 2025.02.05
 */

/* C++ Header */
#include <iostream>
#include <cassert>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>

/* For OpenCV lib */
#include <opencv2/opencv.hpp>

/* For being bridge between ROS Image and OpenCV */
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

/* For ROS image data */
#include "ros/ros.h"
#include "aims_als/Marker.h"
#include "std_msgs/Time.h"
#include "aims_als/ImgWithHeader.h"

/* For ArUco3 Lib */
#include <aruco/fractaldetector.h>

/* For transformation */
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "aims_als/lib/math.h"

using namespace aims_fly;


class FractalDetection
{
    public:
        FractalDetection(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        ~FractalDetection() = default;

        void run() { ros::spin(); };
    
    protected:
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher  pub_marker_;
        ros::Subscriber sub_header_img_;

        image_transport::ImageTransport it_;
        // image_transport::Subscriber img_sub_; 
        // ros::Subscriber sub_img_time_; 

        aruco::FractalDetector fd_;
        
        // parameters
        int _c_width, _c_height, _c_fps;
        float _c_k1, _c_k2, _c_p1, _c_p2, _c_fx, _c_fy, _c_cx, _c_cy;
        double _marker_size;
        std::string _marker_cfg;
        bool _debug, _visualize;

        // variables
        aims_als::Marker fractal_marker_;
        // std_msgs::Time img_time_;

        void initDetector();
        void rodrigues2Quat(const cv::Mat &rvec);
        void setTranslation(const cv::Mat &tvec);
        // void imgCb(const sensor_msgs::ImageConstPtr &msg);
        // void timeCb(const std_msgs::Time::ConstPtr &msg);
        void headerImgCb(const aims_als::ImgWithHeader::ConstPtr &msg);
};



FractalDetection::FractalDetection(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
: nh_(nh), nh_private_(nh_private), it_(nh)
{
    nh_.param<int>("c_width",   _c_width, 1280);
    nh_.param<int>("c_height", _c_height, 800);
    nh_.param<int>("c_fps",       _c_fps, 120);

    nh_.param<float>("c_k1", _c_k1, 0);
    nh_.param<float>("c_k2", _c_k2, 0);
    nh_.param<float>("c_p1", _c_p1, 0);
    nh_.param<float>("c_p2", _c_p2, 0);

    nh_.param<float>("c_fx", _c_fx, 0);
    nh_.param<float>("c_fy", _c_fy, 0);
    nh_.param<float>("c_cx", _c_cx, 0);
    nh_.param<float>("c_cy", _c_cy, 0);

    nh_private_.param<double>("marker_size",              _marker_size,           0.50);
    nh_private_.param<std::string>("marker_configuration", _marker_cfg, "FRACTAL_4L_6");

    nh_private_.param<bool>("debug",         _debug, false);
    nh_private_.param<bool>("visualize", _visualize, false);

    // img_sub_ = it_.subscribe("/camera/image_raw", 1, &FractalDetection::imgCb, this);
    // sub_img_time_ = nh_.subscribe("/aims/image_time", 1, &FractalDetection::timeCb, this, ros::TransportHints().tcpNoDelay());
    sub_header_img_ = nh_.subscribe("/aims/header_image", 1, &FractalDetection::headerImgCb, this, ros::TransportHints().tcpNoDelay());

    pub_marker_ = nh_.advertise<aims_als::Marker>("/aims/fractal_detections", 1);

    initDetector();

    ROS_INFO("Fractal Detection node is initialized..!");
}


void FractalDetection::initDetector()
{
    fd_.setConfiguration(_marker_cfg);
    aruco::CameraParameters cam_params;
    cam_params.CamSize.width = _c_width;
    cam_params.CamSize.height = _c_height; 

    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = _c_fx;
    camera_matrix.at<float>(0, 1) = 0.0f;
    camera_matrix.at<float>(0, 2) = _c_cx;

    camera_matrix.at<float>(1, 0) = 0.0f;
    camera_matrix.at<float>(1, 1) = _c_fy;
    camera_matrix.at<float>(1, 2) = _c_cy;

    camera_matrix.at<float>(2, 0) = 0.0f;
    camera_matrix.at<float>(2, 1) = 0.0f;
    camera_matrix.at<float>(2, 2) = 1.0f;

    cv::Mat dist_coeffs = cv::Mat::zeros(1, 4, CV_32F);
    dist_coeffs.at<float>(0, 0) = _c_k1;
    dist_coeffs.at<float>(0, 1) = _c_k2;
    dist_coeffs.at<float>(0, 2) = _c_p1;
    dist_coeffs.at<float>(0, 3) = _c_p2;

    ROS_INFO("Camera Matrix Size: %d x %d", camera_matrix.rows, camera_matrix.cols);
    ROS_INFO("Distortion Coeffs Size: %d x %d", dist_coeffs.rows, dist_coeffs.cols);

    cam_params.CameraMatrix = camera_matrix.clone();
    cam_params.Distorsion = dist_coeffs.clone();

    fd_.setParams(cam_params, _marker_size);
}

void FractalDetection::rodrigues2Quat(const cv::Mat &rvec)
{
    cv::Mat cvR;
    cv::Rodrigues(rvec, cvR);
    Eigen::Matrix3d R;
    R << cvR.at<double>(0, 0), cvR.at<double>(0, 1), cvR.at<double>(0, 2),
         cvR.at<double>(1, 0), cvR.at<double>(1, 1), cvR.at<double>(1, 2),
         cvR.at<double>(2, 0), cvR.at<double>(2, 1), cvR.at<double>(2, 2);
    
    const Eigen::Vector4d q = R2q(R);
    fractal_marker_.pose_stmp.pose.orientation.w = q(0);
    fractal_marker_.pose_stmp.pose.orientation.x = q(1);
    fractal_marker_.pose_stmp.pose.orientation.y = q(2);
    fractal_marker_.pose_stmp.pose.orientation.z = q(3);

    if (_debug) {
        std::cout << R << std::endl;
    }
}

void FractalDetection::setTranslation(const cv::Mat &tvec)
{
    fractal_marker_.pose_stmp.pose.position.x = tvec.at<double>(0);
    fractal_marker_.pose_stmp.pose.position.y = tvec.at<double>(1);
    fractal_marker_.pose_stmp.pose.position.z = tvec.at<double>(2);

    if (_debug) {
        ROS_INFO("Translation: [%.2f, %.2f, %.2f]", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    }
}

// void FractalDetection::timeCb( const std_msgs::Time::ConstPtr &msg)
// {
//     img_time_.data = msg->data;
// }

// void FractalDetection::imgCb(const sensor_msgs::ImageConstPtr &msg)
// {
    // try {
    //     // Convert the ROS image message to OpenCV format
    //     cv::Mat cv_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    //     if (_debug) {
    //         ROS_INFO("Received image: %d x %d", cv_img.cols, cv_img.rows);
    //     }
        
    //     fractal_marker_.detect = false;

    //     if (cv_img.cols == _c_width) {
    //         if (fd_.detect(cv_img)) {
    //             if (_debug) {
    //                 ROS_INFO("marker is detected");
    //             }
                
    //             if (fd_.poseEstimation()) {
    //                 if (_debug) {
    //                     ROS_INFO("marker Pose is estimated");
    //                 }

    //                 const cv::Mat rvec = fd_.getRvec();
    //                 const cv::Mat tvec = fd_.getTvec();

    //                 rodrigues2Quat(rvec);
    //                 setTranslation(tvec);

    //                 fractal_marker_.pose_stmp.header.stamp = img_time_.data;
    //                 fractal_marker_.detect = true;
    //             }

    //         }
    //     }
    //     // pub_marker_.publish(fractal_marker_);


    //     if (_visualize) {
    //         // if (fd_.detect(cv_img)) {
    //         fd_.drawMarkers(cv_img);
    //         ROS_INFO("Detected Fractal Marker with ID: %d", fd_.getMarkers()[0].id);
    //         ROS_INFO("Pose: [x: %f, y: %f, z: %f]", fractal_marker_.pose_stmp.pose.position.x,
    //                                                 fractal_marker_.pose_stmp.pose.position.y,
    //                                                 fractal_marker_.pose_stmp.pose.position.z);
    //         // }

    //         // Calculate the midpoints
    //         const int mid_x = _c_width / 2;
    //         const int mid_y = _c_height / 2;

    //         // Draw a vertical line at the midpoint
    //         cv::line(cv_img, cv::Point(mid_x, 0), cv::Point(mid_x, _c_height), cv::Scalar(0, 255, 0), 1);

    //         // Draw a horizontal line at the midpoint
    //         cv::line(cv_img, cv::Point(0, mid_y), cv::Point(_c_width, mid_y), cv::Scalar(0, 255, 0), 1);

    //         ROS_INFO("Input Image: (dimension, height, width)=(%d,%d,%d)", cv_img.dims, cv_img.rows, cv_img.cols);
                                                    
    //         cv::imshow("Fractal Marker Detection with Lines", cv_img);
    //         cv::waitKey(1);  
    //     }
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    // }
// }

void FractalDetection::headerImgCb(const aims_als::ImgWithHeader::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // Convert the ROS image message to OpenCV format
        // cv::Mat cv_img = cv_bridge::toCvCopy(msg->image, "bgr8")->image;

        cv_ptr = cv_bridge::toCvCopy(msg->image, "bgr8");
        cv::Mat cv_img = cv_ptr->image;

        if (_debug) {
            ROS_INFO("Received image: %d x %d", cv_img.cols, cv_img.rows);
        }
        
        fractal_marker_.detect = false;

        if (cv_img.cols == _c_width) {
            if (fd_.detect(cv_img)) {
                if (_debug) {
                    ROS_INFO("marker is detected");
                }
                
                if (fd_.poseEstimation()) {
                    if (_debug) {
                        ROS_INFO("marker Pose is estimated");
                    }

                    const cv::Mat rvec = fd_.getRvec();
                    const cv::Mat tvec = fd_.getTvec();

                    rodrigues2Quat(rvec);
                    setTranslation(tvec);

                    fractal_marker_.pose_stmp.header = msg->header;
                    fractal_marker_.detect = true;
                }

            }
        }
        pub_marker_.publish(fractal_marker_);


        if (_visualize) {
            // if (fd_.detect(cv_img)) {
            fd_.drawMarkers(cv_img);
            ROS_INFO("Detected Fractal Marker with ID: %d", fd_.getMarkers()[0].id);
            ROS_INFO("Pose: [x: %f, y: %f, z: %f]", fractal_marker_.pose_stmp.pose.position.x,
                                                    fractal_marker_.pose_stmp.pose.position.y,
                                                    fractal_marker_.pose_stmp.pose.position.z);
            // }

            // Calculate the midpoints
            const int mid_x = _c_width / 2;
            const int mid_y = _c_height / 2;

            // Draw a vertical line at the midpoint
            cv::line(cv_img, cv::Point(mid_x, 0), cv::Point(mid_x, _c_height), cv::Scalar(0, 255, 0), 1);

            // Draw a horizontal line at the midpoint
            cv::line(cv_img, cv::Point(0, mid_y), cv::Point(_c_width, mid_y), cv::Scalar(0, 255, 0), 1);

            ROS_INFO("Input Image: (dimension, height, width)=(%d,%d,%d)", cv_img.dims, cv_img.rows, cv_img.cols);
                                                    
            cv::imshow("Fractal Marker Detection with Lines", cv_img);
            cv::waitKey(1);  
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fractal_detection_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    auto fd_obj = std::make_unique<FractalDetection>(nh, nh_private);

    fd_obj->run();

    return 0;
}
