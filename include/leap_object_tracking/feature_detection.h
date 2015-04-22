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


class detectFeatures{
	
private:

	cv::Mat left;
	cv::Mat right;
	std::vector<cv::KeyPoint> keypointsLeft;
	std::vector<cv::KeyPoint> keypointsRight;
	cv::Mat descriptorLeft;
	cv::Mat descriptorRight;
	std::vector<cv::DMatch > matches;

public:

	detectFeatures(cv::Mat a, cv::Mat b);
	virtual ~detectFeatures() {}

	std::vector<cv::KeyPoint>GetkeypointsLeft(){return keypointsLeft;}
	std::vector<cv::KeyPoint>GetkeypointsRight(){return keypointsRight;}
	cv::Mat GetLeftImage(){return left;};
	cv::Mat GetRightImage(){return right;};

	void SURF_Detector();
	void SIFT_Detector();
	void ORB_Detector();
	void FAST_Detector();
	
	void SURF_Extractor();
	void ORB_Extractor();
	void FREAK_Extractor();
	
	void BruteForce_Matcher();
	
	void Draw_Matches();
	void Draw_Keypoints();
	
	void Show_LeftCam();
	void Show_RightCam();
	
};

#endif
