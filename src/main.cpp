#include "ros/ros.h"
#include "visoRos.h"
// #include "nav_msgs/Odometry.h"
// #include <iostream>
// #include <string>
// #include <vector>
// #include <stdint.h>
// #include "eigen3/Eigen/Dense"
// #include "viso_stereo.h"
// #include <opencv2/opencv.hpp>
// #include <sys/time.h>
// #include <bits/time.h>
// #include <stdlib.h>
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "visoRos_node");
	ros::NodeHandle nh;
// 	VisualOdometryStereo::parameters visualParams;
// 	visualParams.calib.f  = 894.19; // focal length in pixels
// 	visualParams.calib.cu = 660.14; // principal point (u-coordinate) in pixels
// 	visualParams.calib.cv = 261.10; // principal point (v-coordinate) in pixels
// 	visualParams.base     = 0.5707; // baseline in meters
// 	VisualOdometryStereo viso();
	visoRos CVisoRos;

// 	//viso = VisualOdometryStereo(visualParams);
// 	Matrix pose;
// 	pose = Matrix::eye(4);
// 	cv::namedWindow("left",0);
//   cv::namedWindow("right",0);
// 	string dataDir;
// 	dataDir = "/home/xianyu/cmake_ws/libviso2/img/2010_03_09_drive_0023/";
	return 0;
}