// Inclusion guard to prevent this header from being included multiple times
#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H

class detectFeatures{
	
private:

public:

detectFeatures(){}
virtual ~detectFeatures() {}

cv::Mat* SURF_Detector(cv::Mat left, cv::Mat right);
cv::Mat* SIFT_Detector(cv::Mat left, cv::Mat right);
cv::Mat* ORB_Detector(cv::Mat left, cv::Mat right);
cv::Mat* FAST_Detector(cv::Mat left, cv::Mat right);	
};

#endif
