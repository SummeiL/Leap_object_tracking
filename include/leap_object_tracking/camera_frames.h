#ifndef CAMERA_FRAMES_H
#define CAMERA_FRAMES_H
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <highgui.h>

using namespace sensor_msgs;


class CameraFrames{
	
	private:
	
		cv::Mat LeftFrame;
		cv::Mat RightFrame; 
		CameraInfoConstPtr leftCamInfo; 
		CameraInfoConstPtr rightCamInfo;
	
	public:
		
		CameraFrames(){}
		CameraFrames(const CameraFrames&);
		CameraFrames(const ImageConstPtr& Left, const ImageConstPtr& Right, const CameraInfoConstPtr& LeftInfo, const CameraInfoConstPtr& RightInfo);
		
		//operator
		CameraFrames& operator = (const CameraFrames &f);
		
		cv::Mat GetLeftFrame(){ return LeftFrame; }
		cv::Mat GetRightFrame(){ return RightFrame; }
		
		void Show_LeftCam();
		void Show_RightCam();
		const CameraInfoConstPtr GetLeftInfo(){ return leftCamInfo; }
		const CameraInfoConstPtr GetRightInfo(){ return rightCamInfo; }
		
};


#endif
