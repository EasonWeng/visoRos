#ifndef VISOROS_H
#define VISOROS_H
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"
#include "tf/LinearMath/Transform.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include "eigen3/Eigen/Dense"
#include "viso_stereo.h"
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <bits/time.h>
#include <stdlib.h>
using namespace std;
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.002, 0, 0, 0, 0, 0,
    0, 0.002, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
    0, 9999, 0, 0, 0, 0,
    0, 0, 9999, 0, 0, 0,
    0, 0, 0, 9999, 0, 0,
    0, 0, 0, 0, 9999, 0,
    0, 0, 0, 0, 0, 9999 } };
class visoRos
{
public:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_param;
	ros::Publisher odoPub;
	ros::Publisher pathPub;
	ros::Time last_update_time;
 	VisualOdometryStereo::parameters visualParams;
 	//VisualOdometryStereo viso_;
	boost::shared_ptr<VisualOdometryStereo> viso;
	Matrix pose;
	string dataDir;
	string odom_frame_id;
	string base_link_frame_id;
	tf::Transform delta_transform;
	tf::Transform integrated_transform;
	tf::TransformBroadcaster tf_broadcaster;
	nav_msgs::Path path_msg;
	bool pub_odom;
	bool pub_tf;
	bool pub_path;
	visoRos();
	~visoRos();
	void initPubs();	
	void initParams();
	void integrateAndPublish(Matrix delta_T);
};

#endif