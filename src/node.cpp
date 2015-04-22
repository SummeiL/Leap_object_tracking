    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <iostream>
#include <string.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "ros/ros.h"
#include "camera_info_manager/camera_info_manager.h"

//Leap Motion SDK
//#include "/home/juanma/LeapSDK/include/Leap.h"
#include "Leap.h"

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
using namespace sensor_msgs;
using namespace message_filters;

//OPENCV Window names
#define LEFT_WINDOW "Left Raw Image"
#define RIGHT_WINDOW "Right Raw Image"


//Global Variables
cv::Mat mat_img_right;
cv::Mat mat_img_left;


detectFeatures detector;


void ImagesCallback(const ImageConstPtr& imageLeft, const ImageConstPtr& imageRight){
	
	cv_bridge::CvImageConstPtr bridgeLeft;
	cv_bridge::CvImageConstPtr bridgeRight;
	
	Mat *img_keypoints;
	
	try{
		
		bridgeLeft = cv_bridge::toCvCopy(imageLeft, "mono8");
		
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Left ros image.");
		return;
	}
	
	try{
		
		bridgeRight = cv_bridge::toCvCopy(imageRight, "mono8");
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Right ros image.");
		return;
	}
	
	mat_img_right = bridgeRight->image;
	mat_img_left = bridgeLeft->image;
	
	img_keypoints = detector.FAST_Detector(mat_img_left, mat_img_right);

	imshow(LEFT_WINDOW, img_keypoints[0]);
	imshow(RIGHT_WINDOW, img_keypoints[1]);
	
	waitKey(1);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "node");
	ros::NodeHandle nh;
	
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
	
	message_filters::Subscriber<sensor_msgs::Image> imageLeft_sub(nh, "/left/image_raw",1);
	message_filters::Subscriber<sensor_msgs::Image> imageRight_sub(nh, "/right/image_raw",1);
	
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageLeft_sub, imageRight_sub);
	sync.registerCallback(boost::bind(&ImagesCallback, _1, _2));
	

	ros::spin();
  
	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}
