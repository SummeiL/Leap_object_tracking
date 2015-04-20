    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"
#include <cv_bridge/cv_bridge.h>

//Leap Motion SDK
#include "/home/juanma/LeapSDK/include/Leap.h"

//OPENCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
//Catkin
#include <leap_object_tracking/feature_detection.h>
#include <leap_object_tracking/leap_camera.h>

    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

using namespace std;
using namespace cv;

//OPENCV Window names
#define LEFT_WINDOW "Left Raw Image"
#define RIGHT_WINDOW "Right Raw Image"


//Global Variables
cv::Mat mat_img_right;
cv::Mat mat_img_left;


//Subscribers for the topics
ros::Subscriber left_image_subs;
ros::Subscriber right_image_subs;

detectFeatures f;

void leftImageCallback(const sensor_msgs::Image::ConstPtr& msg){

	cv_bridge::CvImageConstPtr bridge;
	Mat img_keypoints;
	
	try{
		
		bridge = cv_bridge::toCvCopy(msg, "mono8");
		
	}
	catch (cv_bridge::Exception& e){
	
		ROS_ERROR("Failed to transform ros image.");
		return;
}
	mat_img_left = bridge->image;
	
	img_keypoints = f.FAST_Detector(mat_img_left);
	
	imshow(LEFT_WINDOW, img_keypoints);
	
	waitKey(1);
}
	
void rightImageCallback(const sensor_msgs::Image::ConstPtr& msg){
	
	cv_bridge::CvImageConstPtr bridge;
	Mat img_keypoints;
	
	try{
		
		bridge = cv_bridge::toCvCopy(msg, "mono8");
	
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform ros image.");
		return;
	}
	
	mat_img_right= bridge->image;
	
	img_keypoints = f.FAST_Detector(mat_img_left);
	
	imshow(RIGHT_WINDOW, img_keypoints);
	
	waitKey(1);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "node");
	ros::NodeHandle n_;
	
	//Create a sample listener and controller
	CameraListener listener;
	Controller controller;
	controller.setPolicy(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_OPTIMIZE_HMD));

	// Have the sample listener receive events from the controller
	controller.addListener(listener);
	controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
	
	//Create OpenCv Windows
	cv::namedWindow(LEFT_WINDOW);
	cv::namedWindow(RIGHT_WINDOW);
	
	//Subscribe to topics
	left_image_subs = n_.subscribe("/left/image_raw", 1,leftImageCallback);
	right_image_subs = n_.subscribe("/right/image_raw", 1,rightImageCallback);

	ros::spin();
  
	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}
