///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <iostream> 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <leap_object_tracking/stereo_camera.h>


//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////

using namespace sensor_msgs;
using namespace std;

StereoCamera::StereoCamera(CameraFrames camFrames){

	Frames = camFrames;


}

StereoCamera::StereoCamera(const StereoCamera &f){
	
	*this = f;
	
}

StereoCamera& StereoCamera::operator = (const StereoCamera &f){
	
	if(this!=&f){
		
		this->Frames = f.Frames;
		this->camModel = f.camModel;
		this->disparity = f.disparity;
		this->stereoproc = f.stereoproc;
		this->disparity_cv = f.disparity_cv;
		this->points3dim = f.points3dim;
		
	}
	
}

void StereoCamera::Camera_Model(){

	camModel.fromCameraInfo(Frames.GetLeftInfo(),Frames.GetRightInfo());

}

void StereoCamera::computeDisparity(){

	stereoproc.processDisparity(Frames.GetLeftFrame(),Frames.GetRightFrame(), camModel, disparity);
	
	disparityTocvFormat();

}

void StereoCamera::disparityTocvFormat(){

	cv_bridge::CvImageConstPtr bridgedisp;

	try{

		bridgedisp = cv_bridge::toCvCopy(disparity.image, sensor_msgs::image_encodings::TYPE_32FC1);
	}

	catch (cv_bridge::Exception& e){

		ROS_ERROR("Failed to transform disparity ros image.");
		return;
	}

	disparity_cv = bridgedisp->image;

}

void StereoCamera::projectpointsTo3d(std::vector<Point2d> points2d){

	for(int i = 0; i < points2d.size(); i++){

		cv::Point2d point2dim;
		point2dim.x = points2d.at(i).x;
		point2dim.y = points2d.at(i).y;

		cv::Point3d point3dim;
		camModel.projectDisparityTo3d(point2dim, disparity_cv.at<float>(point2dim.x, point2dim.y), point3dim);
		
		points3dim.push_back(point3dim);

	}
}

cv::Point3d StereoCamera::projectOnepointTo3d(Point2d point2d, double disparity_value){

		cv::Point3d point3dim;
		camModel.projectDisparityTo3d(point2d,disparity_value, point3dim);
		return point3dim;

}
void StereoCamera::show_disparity(){

	Mat disp_show;
	double minVal; double maxVal;

	minMaxLoc(disparity_cv, &minVal, &maxVal);

	disparity_cv.convertTo(disp_show, CV_8U, 255/(maxVal-minVal));
	imshow("Disparity", disp_show);


}

double StereoCamera::GetdispFromZ(double disp){

	return(camModel.getDisparity(disp));

}