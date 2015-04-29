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
	
	void StereoCamera::projectDisparityKeypointsTo3d(detectFeatures detector){
		
		
		for(int i = 0; i < detector.GetGoodMatches().size(); i++){
		
			cv::Point2d point2dim;
			point2dim.x = detector.GetLeftKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.x;
			point2dim.y = detector.GetLeftKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.y;
		
			cv::Point3d point3dim;
			
			if(disparity_cv.at<float>(point2dim.x, point2dim.y) != -1){
			
				camModel.projectDisparityTo3d(point2dim, disparity_cv.at<float>(point2dim.x, point2dim.y), point3dim);
				points3dim.push_back(point3dim);
			
			}
		}
		
}



	void StereoCamera::processPointCloud(){
		
		cloud.header.frame_id = "leap_optical_frame";
		cloud.height = 1;
		cloud.width = points3dim.size();
		cloud.points.resize (cloud.width * cloud.height);
		cloud.is_dense = false; //there may be invalid points
		
		for(int i = 0; i < points3dim.size(); i++){
			
			
			cloud.points[i].x = points3dim.at(i).x;
			cloud.points[i].y = points3dim.at(i).y;
			cloud.points[i].z = points3dim.at(i).z;
			
			//cout << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << endl;
			
		}
		
		
	}
	
	void StereoCamera::show_disparity(){
		
		Mat disp_show;
		double minVal; double maxVal;

		minMaxLoc(disparity_cv, &minVal, &maxVal);
		
		disparity_cv.convertTo(disp_show, CV_8U, 255/(maxVal-minVal));
		imshow("Disparity", disp_show);
		
		
	}
	
