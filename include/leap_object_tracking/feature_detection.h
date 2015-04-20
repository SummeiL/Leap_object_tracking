// Inclusion guard to prevent this header from being included multiple times
#ifndef FEATURE_DETECTION_H
#define FEATURE_DETECTION_H

class Features{
	
private:

// Our NodeHandle, points to home
ros::NodeHandle nh_;
//Global node handle
ros::NodeHandle n_;

//Subscribers for the topics
ros::Subscriber left_image_subs;
ros::Subscriber right_image_subs;


public:

Features();
virtual ~Features() {}

void leftImageCallback(const sensor_msgs::Image::ConstPtr& msg);	
void rightImageCallback(const sensor_msgs::Image::ConstPtr& msg);
void SURF_Detector(cv::Mat img);
void SIFT_Detector(cv::Mat img);
void ORB_Detector(cv::Mat img);
void FAST_Detector(cv::Mat img);	
};

#endif
