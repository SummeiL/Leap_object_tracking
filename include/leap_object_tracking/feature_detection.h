// Inclusion guard to prevent this header from being included multiple times
#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string.h>
#include <iostream>
#include <leap_object_tracking/camera_frames.h>

class detectFeatures{
	
private:
	
	//Images containers
	cv::Mat first;
	cv::Mat second;
	
	//Descriptors containers
	cv::Mat descriptorFirst;
	cv::Mat descriptorSecond;
	
	//Keypoints vectors
	std::vector<cv::KeyPoint> keypointsFirst;
	std::vector<cv::KeyPoint> keypointsSecond;
	
	//Matches vectors
	std::vector<cv::DMatch > matches;
	std::vector<cv::DMatch> good_matches;


public:
	
	detectFeatures(){}
	detectFeatures(CameraFrames);
	detectFeatures(CameraFrames, CameraFrames);
	virtual ~detectFeatures() {}

	std::vector<cv::KeyPoint>GetFirstKeyPoints(){ return keypointsFirst;}
	std::vector<cv::KeyPoint>GetSecondKeyPoints(){ return keypointsSecond;}
	std::vector<cv::DMatch>GetGoodMatches(){ return good_matches;}
	cv::Mat GetFirstImage(){return first;}
	cv::Mat GetSecondImage(){return second;}
	cv::Point2f GetMatchedPoint(int, int);

	void SURF_Detector();
	void SIFT_Detector();
	void ORB_Detector();
	void FAST_Detector();
	
	void SURF_Extractor();
	void ORB_Extractor();
	void FREAK_Extractor();
	
	void BruteForce_Matcher();
	
	void Draw_Keypoints();
	void Draw_Matches();
	void Draw_GoodMatches();
	

};

#endif
