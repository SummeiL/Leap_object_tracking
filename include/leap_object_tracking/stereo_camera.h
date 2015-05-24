#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <iostream> 
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include "image_geometry/stereo_camera_model.h"
#include <leap_object_tracking/feature_detection.h>
#include <leap_object_tracking/camera_frames.h>


class StereoCamera{

 private:

	CameraFrames Frames;
	image_geometry::StereoCameraModel camModel;
	
	Eigen::MatrixXf H;

	std::vector<cv::Point2f> modelpoints2dleft;
	std::vector<cv::Point2f> modelpoints2dright;
	
 public:
	
	//Constructors and Destructor
	explicit StereoCamera(){}
	explicit StereoCamera(const CameraFrames&);
	StereoCamera(const StereoCamera&);
	~StereoCamera(){}
	
	//Operator
	StereoCamera& operator = (const StereoCamera &);
	
	//Set and Get Methods
	std::vector<cv::Point2f>GetProjectedModelPointsLeft(){ return modelpoints2dleft;}
	std::vector<cv::Point2f>GetProjectedModelPointsRight(){ return modelpoints2dright;}
	void SetFrames(CameraFrames Frames){ this->Frames = Frames;}

	//Methods for computing transformations on the images
	void Camera_Model();
	void FindHomography();
	void ProjectToCameraPlane(std::vector<cv::Point3f>);	
};

#endif
