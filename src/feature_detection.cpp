
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

detectFeatures::detectFeatures(CameraFrames camframe){

	first = camframe.GetLeftFrame();
	second = camframe.GetRightFrame();	
}

detectFeatures::detectFeatures(CameraFrames ActualFrame, CameraFrames OldFrame){
	
	//Constructor for detection in 2 consecutive frames, comparing the first images.
	
	first = ActualFrame.GetLeftFrame();
	second = OldFrame.GetLeftFrame();

}

void detectFeatures::SURF_Detector(){

	int minHessian = 400;

	//Create the SURF detector
	SurfFeatureDetector detector( minHessian );

	//Detect the keypoints using SURF Detector
	detector.detect(first, keypointsFirst);
	detector.detect(second, keypointsSecond);

}

void detectFeatures::SIFT_Detector(){

	//Create the SIFT detector
	SiftFeatureDetector detector;

	//Detect the keypoints using SIFT Detector	
	detector.detect(first, keypointsFirst);
	detector.detect(second, keypointsSecond);

}

void detectFeatures::ORB_Detector(){

	//Create the ORB detector
	OrbFeatureDetector detector;

	//Detect the keypoints using ORB Detector		
	detector.detect(first, keypointsFirst);
	detector.detect(second, keypointsSecond);

}

void detectFeatures::FAST_Detector(){

	//Create the FAST detector
	FastFeatureDetector detector;

	//Detect the keypoints using FAST Detector	
	detector.detect(first, keypointsFirst);
	detector.detect(second, keypointsSecond);

}

void detectFeatures::SURF_Extractor(){

	//Create the SURF extractor
	SurfDescriptorExtractor extractor;

	//Calculate descriptors (feature vectors)
	extractor.compute(first, keypointsFirst, descriptorFirst);
	extractor.compute(second, keypointsSecond, descriptorSecond);

}

void detectFeatures::ORB_Extractor(){

	//Create the ORB extractor
	OrbDescriptorExtractor extractor;

	//Calculate descriptors (feature vectors)
	extractor.compute(first, keypointsFirst, descriptorFirst);
	extractor.compute(second, keypointsSecond, descriptorSecond);

}

void detectFeatures::FREAK_Extractor(){

	//Create the FREAK extractor
	FREAK extractor;

	//Calculate descriptors (feature vectors)
	extractor.compute(first, keypointsFirst, descriptorFirst);
	extractor.compute(second, keypointsSecond, descriptorSecond);

}

void detectFeatures::BruteForce_Matcher(){

	//Create Brute Force Matcher
	BFMatcher matcher(NORM_L2);

	//Mach features between first and second image
	matcher.match(descriptorFirst, descriptorSecond, matches);

	double max_dist = 0; double min_dist = 100;
	double x = 0; double y = 0;

	//Quick calculation of max and min distances between keypoints
	for(int i = 0; i < descriptorFirst.rows; i++){

		double dist = matches[i].distance;

		if(dist < min_dist) min_dist = dist;
		if(dist > max_dist) max_dist = dist;
	}

	//Just select "good" matches
	// i.e whose distance is less than 3*min_dist.

	for(int i = 0; i < descriptorFirst.rows; i++){

		if(matches[i].distance <= max(2*min_dist,0.02)){

			good_matches.push_back(matches[i]);

		}
	}



}


cv::Point2f detectFeatures::GetMatchedPoint(int i, int number){
	
	cv::Point2d point;
	
	if(number == 1){
		
		point = keypointsFirst[good_matches[i].queryIdx].pt;
	}
	
	if(number == 2){
		
		point = keypointsSecond[good_matches[i].trainIdx].pt;
	}
	
	if(number != 1 && number != 2){
		
		ROS_INFO("Invalid argument for the function GetMatchedPoint. Valid arguments are 1 and 2.");
		
	}
	
	
	return point;
	
}

void detectFeatures::Draw_Matches(){

	Mat img_matches;

	drawMatches(first, keypointsFirst, second, keypointsSecond, matches, img_matches);

	imshow("Matches", img_matches);
	waitKey(1);

}

void detectFeatures::Draw_GoodMatches(){

	Mat img_matches;

	drawMatches(first, keypointsFirst, second, keypointsSecond, good_matches, img_matches,
			Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	imshow("Good Matches", img_matches);

	float x = 0; float y = 0;


	waitKey(1);

}
void detectFeatures::Draw_Keypoints(){

	Mat img_keypointsFirst, img_keypointsSecond;

	drawKeypoints( first, keypointsFirst, img_keypointsFirst, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints( second, keypointsSecond, img_keypointsSecond, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

	imshow("First Keypoints", img_keypointsFirst);
	imshow("Second Keypoints", img_keypointsSecond);
	waitKey(1);

}


