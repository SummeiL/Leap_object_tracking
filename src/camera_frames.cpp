
///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////



#include <leap_object_tracking/camera_frames.h>


//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////

CameraFrames::CameraFrames(const sensor_msgs::ImageConstPtr& Left,
		const sensor_msgs::ImageConstPtr& Right, 
		const sensor_msgs::CameraInfoConstPtr& LeftInfo, 
		const sensor_msgs::CameraInfoConstPtr& RightInfo){

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
	
    lowThreshold = 100;
	ratio = 2;
	kernel_size = 3;

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
		this->LeftDistanceFrame = f.LeftDistanceFrame;
		this->RightDistanceFrame = f.RightDistanceFrame;
		this->lowThreshold = f.lowThreshold;
		this->ratio = f.ratio;
		this->kernel_size = f.kernel_size;
	}	
}

/*
	Functions to display the images from the class in an OPENCV window
 */

void CameraFrames::Show_LeftCam(){

	cv::rectangle(LeftFrame, cv::Point(105,80), cv::Point(175,140), cv::Scalar( 255, 255, 255 ), 4);
	cv::imshow("Left Cam", LeftFrame);
	cv::waitKey(1);

}

void CameraFrames::Show_RightCam(){

	cv::rectangle(RightFrame, cv::Point(105,80), cv::Point(175,140), cv::Scalar( 255, 255, 255 ), 4);
	cv::imshow("Right Cam", RightFrame);
	cv::waitKey(1);

}

void CameraFrames::Show_LeftDistanceFrame(){

	cv::imshow("Left Distance Image", LeftDistanceFrame);
	cv::waitKey(1);
}

void CameraFrames::Show_RightDistanceFrame(){

	cv::imshow("Right Distance Image",RightDistanceFrame);
	cv::waitKey(1);
}

/*
	Compute the Canny edge detector for left and right images
	and computes the distance of each pixel to the closest black pixel (edge)
 */

void CameraFrames::EdgeDetector(){

	cv::Point2d point_aux_l;
	cv::Point2d point_aux_r;
	
	double minVal;
	double maxVal;

	cv::Mat dst_left, dst_right;
	cv::Mat LeftCannyFrame;
	cv::Mat RightCannyFrame;

	/// Reduce noise with a kernel 3x3
	cv::blur( LeftFrame, LeftCannyFrame, cv::Size(3,3));
	cv::blur( RightFrame, RightCannyFrame, cv::Size(3,3));

	/// Canny detector Ouput 
	cv::Canny( LeftCannyFrame, LeftCannyFrame, lowThreshold, lowThreshold*ratio, kernel_size, true );	  
	cv::Canny( RightCannyFrame, RightCannyFrame, lowThreshold, lowThreshold*ratio, kernel_size, true );
	
	cv::imshow("Left Canny Image",LeftCannyFrame );
	cv::waitKey(1);

	//Threshold to normalize to 1/0 binary image
	cv::threshold(LeftCannyFrame, dst_left, 254, 1, cv::THRESH_BINARY_INV);
	cv::threshold(RightCannyFrame, dst_right, 254, 1, cv::THRESH_BINARY_INV);

	//Compute the distance to the closest zero pixel
	cv::distanceTransform(dst_left, LeftDistanceFrame, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	cv::distanceTransform(dst_right, RightDistanceFrame, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	
	



}

