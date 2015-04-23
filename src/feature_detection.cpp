
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
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/highgui/highgui.hpp"

//Catking
#include <leap_object_tracking/feature_detection.h>


    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

using namespace std;
using namespace cv;

detectFeatures::detectFeatures(Mat a, Mat b){
	
	left = a;
	right = b;
	
}
std::vector<cv::KeyPoint> detectFeatures::GetRightKeyPoints(){ return keypointsRight;};
std::vector<cv::KeyPoint> detectFeatures::GetLeftKeyPoints(){ return keypointsLeft;};
std::vector<cv::DMatch> detectFeatures::GetGoodMatches(){ return good_matches;}

	
void detectFeatures::SURF_Detector(){
	
	int minHessian = 400;
	
	//Create the SURF detector
	SurfFeatureDetector detector( minHessian );
	
	//Detect the keypoints using SURF Detector
	detector.detect(left, keypointsLeft);
	detector.detect(right, keypointsRight);

}

void detectFeatures::SIFT_Detector(){

	//Create the SIFT detector
	SiftFeatureDetector detector;
	
	//Detect the keypoints using SIFT Detector	
	detector.detect(left, keypointsLeft);
	detector.detect(right, keypointsRight);

}

void detectFeatures::ORB_Detector(){
	 
	//Create the ORB detector
	OrbFeatureDetector detector;

	//Detect the keypoints using ORB Detector		
	detector.detect(left, keypointsLeft);
	detector.detect(right, keypointsRight);

}

void detectFeatures::FAST_Detector(){
 
	//Create the FAST detector
	FastFeatureDetector detector;
	
	//Detect the keypoints using FAST Detector	
	detector.detect(left, keypointsLeft);
	detector.detect(right, keypointsRight);

}

void detectFeatures::SURF_Extractor(){
	
	//Create the SURF extractor
	SurfDescriptorExtractor extractor;
	
	//Calculate descriptors (feature vectors)
	extractor.compute(left, keypointsLeft, descriptorLeft);
	extractor.compute(right, keypointsRight, descriptorRight);
	
}

void detectFeatures::ORB_Extractor(){
	
	//Create the ORB extractor
	OrbDescriptorExtractor extractor;
	
	//Calculate descriptors (feature vectors)
	extractor.compute(left, keypointsLeft, descriptorLeft);
	extractor.compute(right, keypointsRight, descriptorRight);
	
}

void detectFeatures::FREAK_Extractor(){
	
	//Create the FREAK extractor
	FREAK extractor;
	
	//Calculate descriptors (feature vectors)
	extractor.compute(left, keypointsLeft, descriptorLeft);
	extractor.compute(right, keypointsRight, descriptorRight);
	
}

void detectFeatures::BruteForce_Matcher(){
	
	//Create Brute Force Matcher
	BFMatcher matcher(NORM_L2);
	
	//Mach features between left and right image
	matcher.match(descriptorLeft, descriptorRight, matches);
	
	double max_dist = 0; double min_dist = 100;
	double x = 0; double y = 0;
	
	//Quick calculation of max and min distances between keypoints
	for(int i = 0; i < descriptorLeft.rows; i++){
	
		double dist = matches[i].distance;
		
		if(dist < min_dist) min_dist = dist;
		if(dist > max_dist) max_dist = dist;
	}
	
	//Just select "good" matches
	// i.e whose distance is less than 3*min_dist.
	
	for(int i = 0; i < descriptorLeft.rows; i++){
	
		if(matches[i].distance <= max(3*min_dist,0.02)){
			
			good_matches.push_back(matches[i]);
		
		}
	}
	
	
	
}


void detectFeatures::Draw_Matches(){
	
	Mat img_matches;
	
	drawMatches(left, keypointsLeft, right, keypointsRight, matches, img_matches);
	
	imshow("Matches", img_matches);
	waitKey(1);
	
}

void detectFeatures::Draw_GoodMatches(){
	
	Mat img_matches;
	
	drawMatches(left, keypointsLeft, right, keypointsRight, good_matches, img_matches,
	Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	
	imshow("Good Matches", img_matches);
	
	float x = 0; float y = 0;
	
	
	waitKey(1);
	
}
void detectFeatures::Draw_Keypoints(){
	
	Mat img_keypointsLeft, img_keypointsRight;
	
	drawKeypoints( left, keypointsLeft, img_keypointsLeft, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints( right, keypointsRight, img_keypointsRight, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	
	imshow("Left Keypoints", img_keypointsLeft);
	imshow("Right Keypoints", img_keypointsRight);
	waitKey(1);
}

void detectFeatures::Show_LeftCam(){
	
	imshow("Left Cam", left);
	waitKey(1);
	
}

void detectFeatures::Show_RightCam(){
	
	imshow("Right Cam", right);
	waitKey(1);
	
}
