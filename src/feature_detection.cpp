
    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <stdio.h>
#include <iostream> 
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

//OPENCV
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

//Catking
#include <leap_object_tracking/feature_detection.h>


    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

using namespace std;
using namespace cv;

	
Mat detectFeatures::SURF_Detector(Mat img){
	
	int minHessian = 400;
	std::vector<KeyPoint> keypoints;
	Mat img_keypoints; 
	
	//Create the SURF detector
	SurfFeatureDetector detector( minHessian );
	
	//Detect the keypoints using SURF Detector
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	return img_keypoints;
	
}

Mat detectFeatures::SIFT_Detector(Mat img){
	  
	Mat img_keypoints;  
	std::vector<KeyPoint> keypoints;
	
	//Create the SIFT detector
	SiftFeatureDetector detector;
	
	//Detect the keypoints using SIFT Detector	
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	return img_keypoints;
       
}

Mat detectFeatures::ORB_Detector(Mat img){
	  
	Mat img_keypoints;  
	std::vector<KeyPoint> keypoints;
	
	//Create the ORB detector
	OrbFeatureDetector detector;

	//Detect the keypoints using ORB Detector	
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	return img_keypoints;
  
}

Mat detectFeatures::FAST_Detector(Mat img){
  
  	Mat img_keypoints;  
	std::vector<KeyPoint> keypoints;
	
	//Create the FAST detector
	FastFeatureDetector detector;
	
	//Detect the keypoints using FAST Detector	
	detector.detect( img, keypoints);
	
	// Draw keypoints	
	drawKeypoints( img, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  
	return img_keypoints;

}

