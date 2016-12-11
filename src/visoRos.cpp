#include "visoRos.h"

using namespace std;

visoRos::visoRos()
{
	initParams();
	initPubs();
	for(int32_t i = 0; i != 373; i++)
	{
		// input file names
		char base_name[256]; sprintf(base_name,"%06d.png",i);
		string left_img_file_name  = dataDir + "/I1_" + base_name;
		string right_img_file_name = dataDir + "/I2_" + base_name;
		// catch image read/write errors here
		try 
		{
			// load left and right input image
			cv::Mat left_img = cv::imread(left_img_file_name.c_str(),0); // load as black and white image
			cv::Mat right_img = cv::imread(right_img_file_name.c_str(),0);
			//cv::waitKey(1);

			// image dimensions
			int32_t width = left_img.cols;  
			int32_t height = left_img.rows;
			// get pointers to the image data
			uint8_t* left_img_data  = left_img.data;
			uint8_t* right_img_data = right_img.data;
			// status
			cout << "Processing: Frame: " << i;
			// compute visual odometry
			int32_t dims[] = {width,height,width};
			if (viso->process(left_img_data,right_img_data,dims)) 
			{
				// on success, update current pose
				//注意！这里的pose表示的是：k时刻相机在0时刻坐标系下的位姿，所以pose = T_k_to_0, T^k_0 = T^k_k-1 * T^k-1_0, 两边求逆得：T^0_k = T^0_k-1 * (T^k_k-1)^-1 所以有右乘一个逆这个更新式子！
				Matrix delta_T = Matrix::inv(viso->getMotion());
				pose = pose * delta_T;
				integrateAndPublish(delta_T);
				// output some statistics
				double num_matches = viso->getNumberOfMatches();
				double num_inliers = viso->getNumberOfInliers();
				cout << ", Matches: " << num_matches;
				cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
				cout << pose << endl << endl;
				std::vector<Matcher::p_match> p_matches_ = viso->getMatches();
				std::vector<cv::KeyPoint> keypoints1, keypoints2;
				std::vector<cv::DMatch> DMatches;
				for (int32_t i = 0; i != p_matches_.size(); i++)
				{
					keypoints1.push_back(cv::KeyPoint(cv::Point2f(p_matches_[i].u1c, p_matches_[i].v1c),10));
					keypoints2.push_back(cv::KeyPoint(cv::Point2f(p_matches_[i].u2c, p_matches_[i].v2c),10));
					DMatches.push_back(cv::DMatch(i, i, 1));
				}
				cv::drawKeypoints(left_img, keypoints1, left_img, cv::Scalar(255, 255, 255));
// 				cv::imshow("left",left_img);
// 				cv::imshow("right",right_img);
				cv::Mat outputImage;
				cv::drawMatches(left_img, keypoints1, right_img, keypoints2, DMatches, outputImage);
				cv::imshow("left", outputImage);
				if(27 == (uchar)cv::waitKey(1)) break;
				//cv::waitKey(1);
			}
			else 
			{
				cout << " ... failed!" << endl;
			}
		// catch image read errors here
		} 
		catch (...) {
			cerr << "ERROR: Couldn't read input files!" << endl;
			exit(1);
		}
	}
}

visoRos::~visoRos()
{
	cv::destroyAllWindows();
}

void visoRos::initPubs()
{
	odoPub = nh_.advertise<nav_msgs::Odometry>("visoOdom", 10);
	pathPub = nh_.advertise<nav_msgs::Path>("visoPath", 10);
}

void visoRos::initParams()
{
	visualParams.calib.f  = 894.19; // focal length in pixels
	visualParams.calib.cu = 660.14; // principal point (u-coordinate) in pixels
	visualParams.calib.cv = 261.10; // principal point (v-coordinate) in pixels
	visualParams.base     = 0.5707; // baseline in meters
	visualParams.match.nms_n = 3; //3
	visualParams.match.refinement = 1;
	visualParams.ransac_iters = 200; //200
	viso.reset(new VisualOdometryStereo(visualParams));
	pose = Matrix::eye(4);
	delta_transform.setIdentity();
	integrated_transform.setIdentity();
	cv::namedWindow("left",0);
	//cv::namedWindow("right",0);
	odom_frame_id = "/odom";
	base_link_frame_id = "/base_link";
	dataDir = "/home/xianyu/cmake_ws/libviso2/img/2010_03_09_drive_0023/";
	last_update_time = ros::Time::now();
	pub_odom = true;
	pub_tf = false;
	pub_path = true;
	path_msg.poses.clear();
}

void visoRos::integrateAndPublish(Matrix delta_T)
{
	delta_transform.setIdentity();
	tf::Matrix3x3 rot_mat(
		delta_T.val[0][0], delta_T.val[0][1], delta_T.val[0][2],
		delta_T.val[1][0], delta_T.val[1][1], delta_T.val[1][2],
		delta_T.val[2][0], delta_T.val[2][1], delta_T.val[2][2]);
	tf::Vector3 t(delta_T.val[0][3], delta_T.val[1][3], delta_T.val[2][3]);
	delta_transform.setBasis(rot_mat);
	delta_transform.setOrigin(t);
	integrated_transform *= delta_transform;
	nav_msgs::Odometry odo_msg;
	odo_msg.header.stamp = ros::Time::now();
	//std::cout<<odo_msg.header.stamp.toSec();
	odo_msg.header.frame_id = odom_frame_id;
	odo_msg.child_frame_id = base_link_frame_id;
	tf::poseTFToMsg(integrated_transform, odo_msg.pose.pose);
	odo_msg.pose.covariance = STANDARD_POSE_COVARIANCE;
	double dt = (odo_msg.header.stamp - last_update_time).toSec();
	std::cout<<"freq = "<<1.0/dt<<std::endl;
	//double dt = 1;
	last_update_time = odo_msg.header.stamp;
	odo_msg.twist.covariance = STANDARD_TWIST_COVARIANCE;
	odo_msg.twist.twist.linear.x = delta_transform.getOrigin().getX();
	odo_msg.twist.twist.linear.y = delta_transform.getOrigin().getY();
	odo_msg.twist.twist.linear.z = delta_transform.getOrigin().getZ();
	tf::Quaternion delta_q = delta_transform.getRotation();
	tfScalar theta = delta_q.getAngle();
	tf::Vector3 axis_weight = delta_q.getAxis();
	tf::Vector3 angular_rate = axis_weight * theta / dt;
	odo_msg.twist.twist.angular.x = angular_rate.getX();
	odo_msg.twist.twist.angular.y = angular_rate.getY();
	odo_msg.twist.twist.angular.z = angular_rate.getZ();
	path_msg.header = odo_msg.header;
	geometry_msgs::PoseStamped poseStamped;
	poseStamped.header = odo_msg.header;
	poseStamped.pose = odo_msg.pose.pose;
	path_msg.poses.push_back(poseStamped);
	if(pub_odom)
		odoPub.publish(odo_msg);
	if(pub_tf)
		tf_broadcaster.sendTransform(tf::StampedTransform(integrated_transform, odo_msg.header.stamp, odom_frame_id, base_link_frame_id));
	if(pub_path)
		pathPub.publish(path_msg);
	
}






