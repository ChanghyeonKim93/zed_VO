#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Standard includes
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>

// ZED includes
#include <sl/Camera.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>

// Eigen includes
#include <eigen3/Eigen/Dense>


void angle2rotmtx(const Eigen::Vector3d& eulerAngle, Eigen::Matrix3d& rotationMatrix);
void transformCoordinate(float& tx, float& ty, float& tz);


using namespace std;


int main(int argc, char** argv)
{
	// Create a ZED camera object
	sl::Camera zed;


	// Set configuration parameters
	sl::InitParameters init_params;
	init_params.camera_resolution = sl::RESOLUTION_VGA;
	init_params.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
	init_params.coordinate_units  = sl::UNIT_METER;

	init_params.camera_fps = 60;
	init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
	init_params.sdk_verbose = true;


	// Open the camera
	sl::ERROR_CODE err = zed.open(init_params);
	if (err != sl::SUCCESS) {
		std::cout << "Cannot open zed" << std::endl;
		exit(-1);
	}


	// Enable positional tracking with default parameters
	sl::TrackingParameters tracking_parameters;
	tracking_parameters.initial_world_transform = sl::Transform::identity();
	tracking_parameters.enable_spatial_memory = true;


	// Enable motion tracking
	zed.enableTracking(tracking_parameters);


	// Create ROS node to publish VO pose,stereo images
	ros::init(argc, argv, "voTopics");
	ros::NodeHandle nh;

	ros::Publisher voPosePubGeometry = nh.advertise<geometry_msgs::PoseStamped>("/sgpvo/pose", 100);
	geometry_msgs::PoseStamped voData;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher imgPubLeft  = it.advertise("/zed_stereo/left", 1);
	image_transport::Publisher imgPubDepth = it.advertise("/zed_stereo/depth", 1);
	sensor_msgs::ImagePtr imgLeftMsg;
	sensor_msgs::ImagePtr imgDepthMsg;

	// Create sl::Mat object
	sl::Mat image_zed(zed.getResolution(), sl::MAT_TYPE_8U_C4);
	cv::Mat image_ocv(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM_CPU));
	sl::Mat depth_zed(zed.getResolution(), sl::MAT_TYPE_32F_C1);
	cv::Mat depth_ocv(depth_zed.getHeight(), depth_zed.getWidth(), CV_32FC1, depth_zed.getPtr<sl::uchar1>(sl::MEM_CPU));


	// Initialize variables
	sl::Pose zed_pose;
	unsigned long long previous_timestamp, current_timestamp = 0;
	float rx, ry, rz = 0;
	float tx, ty, tz = 0;
	float ox, oy, oz, ow =0;


	// Main loop start
	int image_counter = 1;
	ros::Rate loop_rate(50);
	while(ros::ok()) {
		if (!zed.grab()) {

			// Get camera position in World frame
			sl::TRACKING_STATE tracking_state = zed.getPosition(zed_pose, sl::REFERENCE_FRAME_WORLD);
			int tracking_confidence = zed_pose.pose_confidence;


			// Send OK camera position only
			if (tracking_state == sl::TRACKING_STATE_OK) {

				// Retrieve ZED images
//				zed.retrieveImage(image_zed, sl::VIEW_LEFT);
//				zed.retrieveMeasure(depth_zed, sl::MEASURE_DEPTH);
//				cv::Mat depth_ocv_save;
//				depth_ocv_save = depth_ocv * 5000;
//				depth_ocv_save.convertTo(depth_ocv_save, CV_16UC1);

//				cv::cvtColor(image_ocv, image_ocv, CV_RGBA2RGB);

//				char image_index[255];
//				sprintf(image_index, "%010d.png", image_counter);
//				std::string image_file_name;

				// image_file_name = folder_name_depth + image_index;
				// cv::imwrite(image_file_name, depth_ocv_save);
				// std::cout << image_file_name << std::endl;

				//image_file_name = folder_name_rgb + image_index;
				// cv::imwrite(image_file_name, image_ocv);
				// std::cout << image_file_name << std::endl;


				// image msgs
//				imgLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", image_ocv).toImageMsg();
//				imgPubLeft.publish(imgLeftMsg);
//
//				imgDepthMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_ocv_save).toImageMsg();
//				imgPubDepth.publish(imgDepthMsg);

				// Extract 3x1 rotation from pose
				sl::Vector3<float> rotation = zed_pose.getRotationVector();
				rx = rotation.x;
				ry = rotation.y;
				rz = rotation.z;

				// Extract translation from pose (p_gc)
				sl::Vector3<float> translation = zed_pose.getTranslation();
				tx = translation.tx;
				ty = translation.ty;
				tz = translation.tz;

				// Extract Orientations ( quaternion ) 
				ox = zed_pose.getOrientation().ox;
				oy = zed_pose.getOrientation().oy;
				oz = zed_pose.getOrientation().oz;
				ow = zed_pose.getOrientation().ow;

				// Transform to new inertial coordinate
				transformCoordinate(tx, ty, tz);

				// Extract previous and current timestamp
				previous_timestamp = current_timestamp;
				current_timestamp = zed_pose.timestamp;
				double dt = (double) (current_timestamp - previous_timestamp) * 0.000000001;

				// Display the translation & rotation & timestamp
				printf("Translation: Tx: %.3f, Ty: %.3f, Tz: %.3f, dt: %.3lf [s] \n", tx, ty, tz, dt);
//				printf("Rotation: rx: %.3f, ry: %.3f, rz: %.3f\n", rx, ry, rz);
//				printf("Orientation: ox: %0.3f, oy: %0.3f, oz: %0.3f, ow: %0.3f\n\n",ox,oy,oz,ow);
//				cv::imshow("zed left image", image_ocv);

				// Publish VO pose
				voData.pose.position.x = tx;
				voData.pose.position.y = ty;
				voData.pose.position.z = tz;
				voData.pose.orientation.x = ox;
				voData.pose.orientation.y = oy;
				voData.pose.orientation.z = oz;
				voData.pose.orientation.w = ow;

				voPosePubGeometry.publish(voData);
				image_counter++;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}


	// Disable positional tracking and close the camera
	zed.disableTracking();
	zed.close();
	return 0;
}




void angle2rotmtx(const Eigen::Vector3d& eulerAngle, Eigen::Matrix3d& rotationMatrix)
{
	/**
	  % Project:  Patch-based Illumination invariant Visual Odometry (PIVO)
	  % Function: angle2rotmtx
	  %
	  % Description:
	  %   This function return the rotation matrix rotationMatrix
	  %   [Body frame] = rotationMatrix * [Inertial frame]
	  %   from [phi;theta;psi] angle defined as ZYX sequence to rotation matrix
	  %
	  % Example:
	  %   OUTPUT:
	  %   rotationMatrix = rotation Matrix [3x3] defined as [Body frame] = rotationMatrix * [Inertial frame]
	  %
	  %   INPUT:
	  %   eulerAngle: angle vector composed of [phi;theta;psi]
	  %               phi = Rotation angle along x direction in radians
	  %               theta = Rotation angle along y direction in radians
	  %               psi = Rotation angle along z direction in radians
	  %
	  % NOTE:
	  %
	  % Author: Pyojin Kim
	  % Email: pjinkim1215@gmail.com
	  % Website:
	  %
	  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	  % log:
	  % 2014-08-20:
	  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	  %
	 **/


	// assign roll, pitch, yaw values
	double phi = eulerAngle(0);
	double theta = eulerAngle(1);
	double psi = eulerAngle(2);

	Eigen::Matrix3d rotMtx_Rx = Eigen::Matrix3d::Identity(3,3);
	Eigen::Matrix3d rotMtx_Ry = Eigen::Matrix3d::Identity(3,3);
	Eigen::Matrix3d rotMtx_Rz = Eigen::Matrix3d::Identity(3,3);

	rotMtx_Rx << 1,        0,         0,
			  0, cos(phi), -sin(phi),
			  0, sin(phi),  cos(phi);

	rotMtx_Ry << cos(theta),  0, sin(theta),
			  0,  1,          0,
			  -sin(theta),  0, cos(theta);

	rotMtx_Rz << cos(psi), -sin(psi),   0,
			  sin(psi),  cos(psi),   0,
			  0,         0,   1;

	Eigen::Matrix3d rotMtxBody2Inertial = rotMtx_Rz * rotMtx_Ry * rotMtx_Rx;  // [Inertial frame] = rotMtxBody2Inertial * [Body frame]
	Eigen::Matrix3d rotMtxInertial2Body = rotMtxBody2Inertial.transpose();    //     [Body frame] = rotMtxInertial2Body * [Inertial frame]

	rotationMatrix = rotMtxInertial2Body;
}


void transformCoordinate(float& tx, float& ty, float& tz)
{
	// assign tx, ty, tz in inertial frame
	Eigen::Vector3d zed_translation;
	zed_translation.fill(0);
	zed_translation(0) = tx;
	zed_translation(1) = ty;
	zed_translation(2) = tz;

	// transform from inertial coordinate to new inertial coordinate
	Eigen::Vector3d euler_angle_VICON;
	euler_angle_VICON.fill(0);
	euler_angle_VICON(0) = -30 * (3.14/180);   //  rotation along X axis from inertial to body
	euler_angle_VICON(1) = 0 * (3.14/180);     //  rotation along Y axis from inertial to body
	euler_angle_VICON(2) = 0 * (3.14/180);     //  rotation along Z axis from inertial to body

	Eigen::Matrix3d temp, rotation_matrix_VICON;
	temp.fill(0);
	rotation_matrix_VICON.fill(0);
	angle2rotmtx(euler_angle_VICON, temp);
	rotation_matrix_VICON = temp.inverse();

	// assign tx, ty, tz in new inertial frame
	Eigen::Vector3d zed_translation_new;
	zed_translation_new = rotation_matrix_VICON * zed_translation;
	//tx = (float)zed_translation_new(0);
	//ty = (float)zed_translation_new(1);
	//tz = (float)zed_translation_new(2);
    tx = (float)zed_translation_new(2);
	ty = -(float)zed_translation_new(0);
	tz = -(float)zed_translation_new(1);
}















