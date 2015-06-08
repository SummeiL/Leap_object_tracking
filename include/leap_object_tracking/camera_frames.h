#ifndef CAMERA_FRAMES_H
#define CAMERA_FRAMES_H

///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <highgui.h>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <leap_object_tracking/feature_detection.h>

class CameraFrames{

private:

	cv::Mat LeftFrame;
	cv::Mat RightFrame; 
	cv::Mat LeftDistanceFrame;
	cv::Mat RightDistanceFrame;
	sensor_msgs::CameraInfoConstPtr leftCamInfo; 
	sensor_msgs::CameraInfoConstPtr rightCamInfo;
	
	std::vector<cv::Point2f> modelpoints2dleft;
	std::vector<cv::Point2f> modelpoints2dright;
	cv_bridge::CvImageConstPtr bridgeLeft; 
	cv_bridge::CvImageConstPtr bridgeRight;


public:
	
	//Constructors, and Destructor
	explicit CameraFrames(){}
    CameraFrames(const CameraFrames&);
	explicit CameraFrames(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
	~CameraFrames(){};

	//operator
	CameraFrames& operator = (const CameraFrames&);

	//Get and Set Methods
	cv::Mat GetLeftFrame(){ return LeftFrame; }
	cv::Mat GetRightFrame(){ return RightFrame; }

	cv::Mat GetLeftDistanceFrame(){return LeftDistanceFrame;}
	cv::Mat GetRightDistanceFrame(){return RightDistanceFrame;}

	const sensor_msgs::CameraInfoConstPtr GetLeftInfo(){ return leftCamInfo; }
	const sensor_msgs::CameraInfoConstPtr GetRightInfo(){ return rightCamInfo; }
	
	std::vector<cv::Point2f> GetProjectedModelPointsLeft(){return modelpoints2dleft;}
	std::vector<cv::Point2f> GetProjectedModelPointsRight(){return modelpoints2dright;}


	//Methods for the adquired images
	void EdgeDetector();
	void ProjectToCameraPlane(Eigen::MatrixXf cloud);

	void Show_LeftCam();
	void Show_RightCam();
	void Show_LeftDistanceFrame();
	void Show_RightDistanceFrame();		
};

#endif
