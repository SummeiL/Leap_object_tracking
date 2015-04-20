//include headers from ROS
#include <stdio.h>
#include <iostream> 
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//EIGEN
#include <Eigen/Eigen>
//OPENCV
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

//OPENCV Window names
#define LEFT_WINDOW "Left Raw Image"
#define RIGHT_WINDOW "Right Raw Image"
#include <leap_object_tracking/feature_detection.h>

//Global Variables
cv::Mat mat_img_right;
cv::Mat mat_img_left;

using namespace std;
using namespace cv;

Features::Features(){
	
nh_ = ros::NodeHandle("~");
n_ = ros::NodeHandle();

//Subscribe to topics
left_image_subs = n_.subscribe("/left/image_raw", 1, &Features::leftImageCallback, this);
right_image_subs = n_.subscribe("/right/image_raw", 1, &Features::rightImageCallback, this);


//Create OpenCv Windows
cv::namedWindow(LEFT_WINDOW);
cv::namedWindow(RIGHT_WINDOW);

}

void Features::leftImageCallback(const sensor_msgs::Image::ConstPtr& msg){

	cv_bridge::CvImageConstPtr bridge;
	try{
		
		bridge = cv_bridge::toCvCopy(msg, "mono8");
		
	}
	catch (cv_bridge::Exception& e){
	
		ROS_ERROR("Failed to transform ros image.");
		return;
}
	mat_img_left = bridge->image;
	//cv::imshow(LEFT_WINDOW,bridge->image);
	ORB_Detector(mat_img_left);
	cv::waitKey(1);

}
	
void Features::rightImageCallback(const sensor_msgs::Image::ConstPtr& msg){
	
	cv_bridge::CvImageConstPtr bridge;
	try{
		
		bridge = cv_bridge::toCvCopy(msg, "mono8");
	
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform ros image.");
		return;
	}
	
	mat_img_right= bridge->image;
	FAST_Detector(mat_img_right);

	//cv::imshow(RIGHT_WINDOW,bridge->image);
	cv::waitKey(1);

}



	
void Features::SURF_Detector(Mat img){
	
	int minHessian = 400;
	std::vector<KeyPoint> keypoints;
	Mat img_keypoints; 
	
	//Create the SURF detector
	SurfFeatureDetector detector( minHessian );
	
	//Detect the keypoints using SURF Detector
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	imshow(LEFT_WINDOW, img_keypoints);

}

void Features::SIFT_Detector(Mat img){
	  
	Mat img_keypoints;  
	std::vector<KeyPoint> keypoints;
	
	//Create the SIFT detector
	SiftFeatureDetector detector;
	
	//Detect the keypoints using SIFT Detector	
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	imshow(LEFT_WINDOW, img_keypoints);
       
}

void Features::ORB_Detector(Mat img){
	  
	Mat img_keypoints;  
	std::vector<KeyPoint> keypoints;
	
	//Create the ORB detector
	OrbFeatureDetector detector;

	//Detect the keypoints using ORB Detector	
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	imshow(LEFT_WINDOW, img_keypoints);
  
}

void Features::FAST_Detector(Mat img){
  
  	Mat img_keypoints;  
	std::vector<KeyPoint> keypoints;
	
	//Create the FAST detector
	FastFeatureDetector detector;
	
	//Detect the keypoints using FAST Detector	
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	imshow(RIGHT_WINDOW, img_keypoints);

}

