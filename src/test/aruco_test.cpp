/**
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 */
#include <iostream>

#include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <aruco/fractaldetector.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;

void printDebug(int num)
{
    ROS_INFO("H%d",num);
}

cv::Mat latest_image;  // Global variable to store the latest image

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Convert the ROS image message to OpenCV format
        latest_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        ROS_INFO("Received image: %d x %d", latest_image.cols, latest_image.rows);

        // Optional: Show the image
        //cv::imshow("Camera Image", latest_image);
        // cv::waitKey(1);  // Needed for imshow to work properly
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


void rodrigues2Quat(const cv::Mat &rvec)
{
    cv::Mat cvR;
    cv::Rodrigues(rvec, cvR);
    Eigen::Matrix3d R;
    R << cvR.at<double>(0, 0), cvR.at<double>(0, 1), cvR.at<double>(0, 2),
         cvR.at<double>(1, 0), cvR.at<double>(1, 1), cvR.at<double>(1, 2),
         cvR.at<double>(2, 0), cvR.at<double>(2, 1), cvR.at<double>(2, 2);
    
    std::cout << R << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_test_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/camera/image_raw", 1, imageCallback);


    aruco::FractalDetector fd_;
    
    printDebug(1);
    fd_.setConfiguration("FRACTAL_4L_6");
    printDebug(2);


    aruco::CameraParameters cam_params;
    cam_params.CamSize.width = 800;
    printDebug(3);

    cam_params.CamSize.height = 600; 
    printDebug(3);


    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = 676.641462f;
    camera_matrix.at<float>(0, 1) = 0.0f;
    camera_matrix.at<float>(0, 2) = 399.500000f;

    camera_matrix.at<float>(1, 0) = 0.0f;
    camera_matrix.at<float>(1, 1) = 678.702192f;
    camera_matrix.at<float>(1, 2) = 299.500000f;

    camera_matrix.at<float>(2, 0) = 0.0f;
    camera_matrix.at<float>(2, 1) = 0.0f;
    camera_matrix.at<float>(2, 2) = 1.0f;

    cv::Mat dist_coeffs = cv::Mat::zeros(1, 4, CV_32F);
    dist_coeffs.at<float>(0, 0) = 0.016492f;
    dist_coeffs.at<float>(0, 1) = -0.004420f;
    dist_coeffs.at<float>(0, 2) = 0.002682f;
    dist_coeffs.at<float>(0, 3) = -0.011383f;

    ROS_INFO("Camera Matrix Size: %d x %d", camera_matrix.rows, camera_matrix.cols);
    ROS_INFO("Distortion Coeffs Size: %d x %d", dist_coeffs.rows, dist_coeffs.cols);

    cam_params.CameraMatrix = camera_matrix.clone();
    cam_params.Distorsion = dist_coeffs.clone();


    fd_.setParams(cam_params, 0.175);

    ros::Rate rate(1);
    while (ros::ok()) {
        if (latest_image.cols == 800) {
            printDebug(10);
            if (fd_.detect(latest_image)) {
                ROS_INFO("marker is detected");
                if (fd_.poseEstimation()) {
                    const cv::Mat rvec = fd_.getRvec();
                    const cv::Mat tvec = fd_.getTvec();
                    ROS_INFO("marker Pose is estimated");

                    rodrigues2Quat(rvec);
                    ROS_INFO("Translation: [%.2f, %.2f, %.2f]", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

                }

            }
        }



        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
