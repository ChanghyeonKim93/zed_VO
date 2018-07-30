#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>

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

class VOAT {

    public:
    VOAT();
    ~VOAT();
    void initialize_cameras();
	void finish_cameras();
    void start_vo();
    void finish_vo();

    void publish_pose();

	bool get_toggle();

	//callbacks.
	public:
	bool toggle_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);


    // related to rotation matrix
    public:
    void angle2rotmtx(const Eigen::Vector3d& eulerAngle, Eigen::Matrix3d& rotationMatrix);
    void transformCoordinate(float& tx, float& ty, float& tz);
    

    private: 
	// node handler, publishers, and subscribers
    ros::NodeHandle                 nh;
    ros::Publisher                  voPosePubGeometry;
ros::ServiceServer              toggleServiceServer;

	//image_transport::ImageTransport it;
	//image_transport::Publisher      imgPubLeft, imgPubRight;

	// pose and status
	geometry_msgs::PoseStamped vo_data;
	bool vo_toggle;
	unsigned long long previous_timestamp, current_timestamp;

	float rx, ry, rz;
	float tx, ty, tz;
	float ox, oy, oz, ow;

    // related to zed cameras
    sl::Camera             zed; // create a ZED camera object
  	sl::InitParameters     init_params;
	sl::TrackingParameters tracking_params;
	sl::Pose               zed_pose;

	// images
	//cv::Mat leftImg, rightImg;
};

VOAT::VOAT() {
	// Service
	this->toggleServiceServer = this->nh.advertiseService("sgpvo/vo_toggle", &VOAT::toggle_callback, this);

	// Publisher
	this->voPosePubGeometry = this->nh.advertise<geometry_msgs::PoseStamped> ("sgpvo/pose", 1);

	// initialize variables
	this->vo_toggle = false;
	this->previous_timestamp = 0.0;
	this->current_timestamp  = 0.0;

	printf("VO constructor.\n");
}

VOAT::~VOAT() {
	if(this->vo_toggle == true) VOAT::finish_vo();
	VOAT::finish_cameras();
}


void VOAT::initialize_cameras() {
    // Set configuration parameters
    this->init_params.camera_resolution = sl::RESOLUTION_VGA;
	this->init_params.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
	this->init_params.coordinate_units  = sl::UNIT_METER;
	
	this->init_params.camera_fps        = 60;
	this->init_params.depth_mode        = sl::DEPTH_MODE_PERFORMANCE;
	this->init_params.sdk_verbose       = true;

	// Open the camera
    sl::ERROR_CODE err = this->zed.open(this->init_params);
	if (err != sl::SUCCESS) 
    {
		printf("ERROR : cannot open zed cameras..\n");
		exit(1);
	}
	printf("zed cameras are turned on.\n");
}

void VOAT::finish_cameras() {
	this->zed.close();
	printf("zed cameras are turned off.\n");
}

void VOAT::start_vo() {
	// Enable positional tracking with default parameters
	this->tracking_params.initial_world_transform = sl::Transform::identity();
	this->tracking_params.enable_spatial_memory   = true;

	// Enable motion tracking
	this->zed.enableTracking(this->tracking_params);
    printf("Start VO. \n");
}

void VOAT::finish_vo() {
    // Disable positional tracking and close the camera
	this->zed.disableTracking();
	printf("Finish VO. \n");
}

void VOAT::publish_pose() {
	if(!this->zed.grab()) 
	{
		// Get camera position in World frame
		sl::TRACKING_STATE tracking_state = this->zed.getPosition(this->zed_pose, sl::REFERENCE_FRAME_WORLD);
		int tracking_confidence = this->zed_pose.pose_confidence;
		if (tracking_state == sl::TRACKING_STATE_OK) {
			// Extract 3x1 rotation from pose
			sl::Vector3<float> rotation = zed_pose.getRotationVector();
			this->rx = rotation.x;
			this->ry = rotation.y;
			this->rz = rotation.z;

			// Extract translation from pose (p_gc)
			sl::Vector3<float> translation = zed_pose.getTranslation();
			this->tx = translation.tx;
			this->ty = translation.ty;
			this->tz = translation.tz;

			// Extract Orientations ( quaternion ) 
	    	this->ox = zed_pose.getOrientation().ox;
			this->oy = zed_pose.getOrientation().oy;
			this->oz = zed_pose.getOrientation().oz;
			this->ow = zed_pose.getOrientation().ow;

			// Transform to new inertial coordinate
			this->transformCoordinate(this->tx, this->ty, this->tz);

			// Display the translation & rotation & timestamp

			this->previous_timestamp = this->current_timestamp;
			this->current_timestamp = this->zed_pose.timestamp;
			double dt = (double) (this->current_timestamp - this->previous_timestamp) * 0.000000001;

			printf("Translation: Tx: %.3f, Ty: %.3f, Tz: %.3f, dt: %.3lf \n", this->tx,this-> ty, this->tz, dt);
			
			// Publish VO pose
			this->vo_data.pose.position.x = this->tx;
			this->vo_data.pose.position.y = this->ty;
			this->vo_data.pose.position.z = this->tz;
			this->vo_data.pose.orientation.x = this->ox;
			this->vo_data.pose.orientation.y = this->oy;
			this->vo_data.pose.orientation.z = this->oz;
			this->vo_data.pose.orientation.w = this->ow;

			this->voPosePubGeometry.publish(this->vo_data);
		}
	}

}

bool VOAT::get_toggle() {
       return this->vo_toggle;
}

bool VOAT::toggle_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	if(req.data == true) {
		this->start_vo();       // start vo
		this->vo_toggle = true; // go to VO-on mode.
		res.success = true;     // send "ACK" to GCS.
		ROS_INFO_STREAM("VO-on toggled by Service (VO).");
	} else if (req.data == false) {
		//this->vo_toggle = STATE_VO;    // go to VO    mode
		res.success = false;
		ROS_INFO_STREAM("VO-off toggled by Service (VO) : not implemented.");
	}
}




void VOAT::angle2rotmtx(const Eigen::Vector3d& eulerAngle, Eigen::Matrix3d& rotationMatrix)
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


void VOAT::transformCoordinate(float& tx, float& ty, float& tz)
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
	this->angle2rotmtx(euler_angle_VICON, temp);
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















