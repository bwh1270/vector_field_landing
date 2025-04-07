#include <iostream>
#include "opencv2/opencv.hpp"
#include <ros/ros.h>


int main(int argc, char** argv) {

	ros::init(argc, argv, "opencv_version_test");
	ros::NodeHandle nh;
	std::cout << "OpenCV version: [" << CV_VERSION << "]" << std::endl;
 
	return 0;
}