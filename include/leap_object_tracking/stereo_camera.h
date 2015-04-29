#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include "image_geometry/stereo_camera_model.h"
#include "stereo_image_proc/processor.h"
#include <leap_object_tracking/camera_frames.h>
#include <leap_object_tracking/feature_detection.h>
#include "pcl_ros/point_cloud.h"

using namespace cv;
using namespace sensor_msgs;

class StereoCamera{

 private:

	CameraFrames Frames;
	image_geometry::StereoCameraModel camModel;
	stereo_msgs::DisparityImage disparity;
	stereo_image_proc::StereoProcessor stereoproc;
	Mat disparity_cv;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	vector<cv::Point3d> points3dim;
	
 public:

	StereoCamera(CameraFrames);
	void Camera_Model();
	void show_disparity();
	void computeDisparity();
	void disparityTocvFormat();
	void projectDisparityKeypointsTo3d(detectFeatures);
	void processPointCloud();
	pcl::PointCloud<pcl::PointXYZ> GetCloud(){ return cloud; }
};

#endif
