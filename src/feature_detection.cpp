
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

	
Mat* detectFeatures::SURF_Detector(Mat imgLeft, Mat imgRight){
	
	int minHessian = 400;
	
	static Mat img_keypoints[2]; 
	Mat *pointer;
	std::vector<KeyPoint> keypointsLeft, keypointsRight;
	
	//Create the SURF detector
	SurfFeatureDetector detector( minHessian );
	
	//Detect the keypoints using SURF Detector
	detector.detect(imgLeft, keypointsLeft);
	detector.detect(imgRight, keypointsRight);
	
	drawKeypoints( imgLeft, keypointsLeft, img_keypoints[0], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( imgRight, keypointsRight, img_keypoints[1], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	
	pointer = img_keypoints;
	
	return pointer;
}

Mat* detectFeatures::SIFT_Detector(Mat imgLeft, Mat imgRight){
	  
	static Mat img_keypoints[2]; 
	Mat *pointer;
	std::vector<KeyPoint> keypointsLeft, keypointsRight;
	
	//Create the SIFT detector
	SiftFeatureDetector detector;
	
	//Detect the keypoints using SIFT Detector	
	detector.detect(imgLeft, keypointsLeft);
	detector.detect(imgRight, keypointsRight);
	
	// Draw keypoints	
	drawKeypoints( imgLeft, keypointsLeft, img_keypoints[0], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( imgRight, keypointsRight, img_keypoints[1], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	
	pointer = img_keypoints;
	
	return pointer;
       
}

Mat* detectFeatures::ORB_Detector(Mat imgLeft, Mat imgRight){
	  
	static Mat img_keypoints[2]; 
	Mat *pointer;
	std::vector<KeyPoint> keypointsLeft, keypointsRight;
	
	//Create the ORB detector
	OrbFeatureDetector detector;

	//Detect the keypoints using ORB Detector		
	detector.detect(imgLeft, keypointsLeft);
	detector.detect(imgRight, keypointsRight);
	
	// Draw keypoints	
	drawKeypoints( imgLeft, keypointsLeft, img_keypoints[0], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( imgRight, keypointsRight, img_keypoints[1], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	
	pointer = img_keypoints;
	
	return pointer;
}

Mat* detectFeatures::FAST_Detector(Mat imgLeft, Mat imgRight){
  
	static Mat img_keypoints[2]; 
	Mat *pointer;
	std::vector<KeyPoint> keypointsLeft, keypointsRight;
	
	//Create the FAST detector
	FastFeatureDetector detector;
	
	//Detect the keypoints using FAST Detector	
	detector.detect(imgLeft, keypointsLeft);
	detector.detect(imgRight, keypointsRight);
	
	// Draw keypoints	
	drawKeypoints( imgLeft, keypointsLeft, img_keypoints[0], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    drawKeypoints( imgRight, keypointsRight, img_keypoints[1], Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	
	pointer = img_keypoints;
	
	return pointer;

}

