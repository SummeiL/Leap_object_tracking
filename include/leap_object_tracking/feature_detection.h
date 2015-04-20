// Inclusion guard to prevent this header from being included multiple times
#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H

class detectFeatures{
	
private:

public:

detectFeatures(){}
virtual ~detectFeatures() {}

cv::Mat SURF_Detector(cv::Mat img);
cv::Mat SIFT_Detector(cv::Mat img);
cv::Mat ORB_Detector(cv::Mat img);
cv::Mat FAST_Detector(cv::Mat img);	
};

#endif
