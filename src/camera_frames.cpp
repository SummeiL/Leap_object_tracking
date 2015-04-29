
    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

//Catking
#include <leap_object_tracking/camera_frames.h>


    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////



CameraFrames::CameraFrames(const ImageConstPtr& Left, const ImageConstPtr& Right, const CameraInfoConstPtr& LeftInfo, const CameraInfoConstPtr& RightInfo){
	
	leftCamInfo = LeftInfo; 
	rightCamInfo = RightInfo;
		 
	cv_bridge::CvImageConstPtr bridgeLeft; 
	cv_bridge::CvImageConstPtr bridgeRight;
		
	try{
		
		bridgeLeft = cv_bridge::toCvCopy(Left, "mono8");
		
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Left ros image.");
		return;
	}
	
	try{
		
		bridgeRight = cv_bridge::toCvCopy(Right, "mono8");
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Right ros image.");
		return;
	}
		
	LeftFrame = bridgeLeft->image;
	RightFrame = bridgeRight->image;	 
		 
		 
}

CameraFrames::CameraFrames(const CameraFrames &f){
	
	*this = f;
	
}
CameraFrames& CameraFrames::operator = (const CameraFrames &f){
	
	if(this!=&f){
		
		this->LeftFrame = f.LeftFrame;
		this->RightFrame = f.RightFrame;
		this->leftCamInfo = f.leftCamInfo;
		this->rightCamInfo = f.rightCamInfo;
		
	}
	
}







