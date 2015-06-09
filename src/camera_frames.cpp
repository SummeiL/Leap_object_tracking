
///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////



#include <leap_object_tracking/camera_frames.h>

#include "opencv2/calib3d/calib3d.hpp"

#include <leap_object_tracking/GetTime.h>


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

	}	
}

/*
	Functions to display the images from the class in an OPENCV window
 */

void CameraFrames::Show_LeftCam(){

	//cv::rectangle(LeftFrame, cv::Point(105,80), cv::Point(175,140), cv::Scalar( 255, 255, 255 ), 4);
	cv::imshow("Left Cam", LeftFrame);
	cv::waitKey(1);

}

void CameraFrames::Show_RightCam(){

	//cv::rectangle(RightFrame, cv::Point(105,80), cv::Point(175,140), cv::Scalar( 255, 255, 255 ), 4);
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
	
	int lowThreshold = 100;
	int ratio = 2;
	int kernel_size = 3;

	cv::Mat dst_left, dst_right;
	cv::Mat LeftCannyFrame;
	cv::Mat RightCannyFrame;

	/// Reduce noise with a kernel 3x3
	cv::blur( LeftFrame, LeftCannyFrame, cv::Size(3,3));
	cv::blur( RightFrame, RightCannyFrame, cv::Size(3,3));

	/// Canny detector Ouput 
	cv::Canny( LeftCannyFrame, LeftCannyFrame, lowThreshold, lowThreshold*ratio, kernel_size, true );	  
	cv::Canny( RightCannyFrame, RightCannyFrame, lowThreshold, lowThreshold*ratio, kernel_size, true );

	//Threshold to normalize to 1/0 binary image
	cv::threshold(LeftCannyFrame, dst_left, 254, 1, cv::THRESH_BINARY_INV);
	cv::threshold(RightCannyFrame, dst_right, 254, 1, cv::THRESH_BINARY_INV);
	
	cv::Mat tmpleft, tmpright;
	//Compute the distance to the closest zero pixel
	cv::distanceTransform(dst_left, tmpleft, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	cv::distanceTransform(dst_right, tmpright,  CV_DIST_L2, CV_DIST_MASK_PRECISE);
	
	cv::normalize(tmpleft, LeftDistanceFrame, 0, 100, cv::NORM_MINMAX, CV_32FC1);
	cv::normalize(tmpright, RightDistanceFrame, 0, 100, cv::NORM_MINMAX, CV_32FC1);
}


void CameraFrames::ProjectToCameraPlane(Eigen::MatrixXf cloud){

	Eigen::MatrixXf P_L(3,4);
	Eigen::MatrixXf P_R(3,4);
	Eigen::MatrixXf Point2dimensions_right(3,cloud.cols());
	Eigen::MatrixXf Point2dimensions_left(3,cloud.cols());
	Eigen::MatrixXf u = Eigen::MatrixXf::Ones(1,cloud.cols());
	Eigen::MatrixXf translation_l(4,1);
	Eigen::MatrixXf translation_r(4,1);
	
	cv::Point2f aux_l(0,0);
	cv::Point2f aux_r(0,0);
	
	modelpoints2dright.clear();
	modelpoints2dleft.clear();
	
	//The Projection matrix projects points in the camera frame.
	//Construct the Projection Matrix of the Left Camera
	P_L(0,0) = leftCamInfo->P[0];	P_L(1,0) = leftCamInfo->P[4]; 	P_L(2,0) = leftCamInfo->P[8];
	P_L(0,1) = leftCamInfo->P[1]; 	P_L(1,1) = leftCamInfo->P[5]; 	P_L(2,1) = leftCamInfo->P[9];
	P_L(0,2) = leftCamInfo->P[2]; 	P_L(1,2) = leftCamInfo->P[6]; 	P_L(2,2) = leftCamInfo->P[10];
	P_L(0,3) = leftCamInfo->P[3]; 	P_L(1,3) = leftCamInfo->P[7]; 	P_L(2,3) = leftCamInfo->P[11];
	
	//Construct the Projection Matrix of the Right Camera	
	P_R(0,0) = rightCamInfo->P[0]; 	P_R(1,0) = rightCamInfo->P[4]; 	P_R(2,0) = rightCamInfo->P[8];
	P_R(0,1) = rightCamInfo->P[1]; 	P_R(1,1) = rightCamInfo->P[5]; 	P_R(2,1) = rightCamInfo->P[9];
	P_R(0,2) = rightCamInfo->P[2]; 	P_R(1,2) = rightCamInfo->P[6]; 	P_R(2,2) = rightCamInfo->P[10];
	P_R(0,3) = rightCamInfo->P[3]; 	P_R(1,3) = rightCamInfo->P[7];	P_R(2,3) = rightCamInfo->P[11];
	
	/*Project the 3D point onto the camera planes [u v w] = ProjMat * (3DPoints+Translation).
	 Translation is neccesary to put the 3D points on the corresponding camera frame.
	 Baseline = 0.04 and global frame in the midle of the baseline.  */
	
	translation_l << 0, 0, 0, 0;
	translation_r << -0.02, 0, 0, 0;
	Point2dimensions_left = P_L*(cloud+(translation_l*u));
	Point2dimensions_right = P_R*(cloud+(translation_r*u));
	
	//Divide by the Homogeneous Coordinates to get the 2D Points
	Point2dimensions_left.row(0) = Point2dimensions_left.row(0).cwiseQuotient(Point2dimensions_left.row(2));
	Point2dimensions_left.row(1) = Point2dimensions_left.row(1).cwiseQuotient(Point2dimensions_left.row(2));
	
	Point2dimensions_right.row(0) = Point2dimensions_right.row(0).cwiseQuotient(Point2dimensions_right.row(2));
	Point2dimensions_right.row(1) = Point2dimensions_right.row(1).cwiseQuotient(Point2dimensions_right.row(2));


		for(int i = 0; i < cloud.cols(); i++){

			aux_l.x = Point2dimensions_left(0,i);
			aux_l.y = Point2dimensions_left(1,i);
			
			aux_r.x = Point2dimensions_right(0,i);
			aux_r.y = Point2dimensions_right(1,i);
					
			if(aux_l.x < 280 && aux_l.x >=0 && aux_r.x < 280 && aux_r.x >= 0 && aux_l.y < 220 && aux_l.y >= 0 && aux_r.y < 220 && aux_r.y >=0 ){
				
				modelpoints2dleft.push_back(aux_l);
				modelpoints2dright.push_back(aux_r);
				
			}
		}
/*		cv::Point center(floor(aux_l.x), floor(aux_l.y));
		cv::circle(LeftFrame,center,0.1,cv::Scalar(255,255,255),-1);
		cv::Point center2(floor(aux_l.x), floor(aux_l.y));
		cv::circle(RightFrame,center2,0.1,cv::Scalar(255,255,255),-1);
		cv::imshow("pointsleft", LeftFrame);
		cv::imshow("pointsright", RightFrame);
		cv::waitKey(1);*/
		

}

