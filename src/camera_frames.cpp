
///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////



#include <leap_object_tracking/camera_frames.h>

#include "opencv2/calib3d/calib3d.hpp"



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
		this->H = f.H;
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

	//Compute the distance to the closest zero pixel
	cv::distanceTransform(dst_left, LeftDistanceFrame, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	cv::distanceTransform(dst_right, RightDistanceFrame, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	
}

/*
	Computes the Homography matrix with keypoint detectors
 */

void CameraFrames::Homography(){

	std::vector<cv::Point2f> left;
	std::vector<cv::Point2f> right;

	detectFeatures detect(LeftFrame, RightFrame);

	//Detect Keypoints in both images left/right
	detect.FAST_Detector();

	//Extract Features from the keypoints left/right
	detect.SURF_Extractor();

	//Match the keypoints of the two images
	detect.BruteForce_Matcher();

	for(int i = 0; i < detect.GetGoodMatches().size(); i++){

		left.push_back(detect.GetMatchedPoint(i, 1));
		right.push_back(detect.GetMatchedPoint(i, 2));		

	}
	
	detect.Draw_GoodMatches();

	//findHomography needs at least 4 points to comput it, and almost 10 to be accurate
	if(left.size() < 4 || right.size() < 4){

		H.setZero();

	}else{
		
		//Compute the Homography image between left and right images
		cv::Mat_<float> H_aux =  cv::findHomography(left, right, CV_RANSAC);
		
		//Transform it to Eigen Matrix
		cv::cv2eigen(H_aux, H);
	}
}

void CameraFrames::ProjectToCameraPlane(std::vector<cv::Point3f> cloud){

	Eigen::MatrixXf A(3,4);
	Eigen::MatrixXf Point2dimensions_right(1,3);
	Eigen::MatrixXf Point3dimensions_left(1,4);
	Eigen::MatrixXf Point2dimensions_left(1,3);

	modelpoints2dright.clear();
	modelpoints2dleft.clear();

	//Construct the Projection Matrix of the Left Camera
	A(0,0) = leftCamInfo->P[0];
	A(0,1) = leftCamInfo->P[1];
	A(0,2) = leftCamInfo->P[2];
	A(0,3) = leftCamInfo->P[3];
	A(1,0) = leftCamInfo->P[4];
	A(1,1) = leftCamInfo->P[5];
	A(1,2) = leftCamInfo->P[6];
	A(1,3) = leftCamInfo->P[7];
	A(2,0) = leftCamInfo->P[8];
	A(2,1) = leftCamInfo->P[9];
	A(2,2) = leftCamInfo->P[10];
	A(2,3) = leftCamInfo->P[11];
	//std::cout << A << std::endl;

	if (!H.isZero(0.000000)){
		
		for(int i = 0; i < cloud.size(); i++){
			cv::Point2f aux_l(0,0);
			cv::Point2f aux_r(0,0);
			
			//Store the 3D point in a vector and convert it to Homogeneus coordinates [X Y Z 1]
			//The Projection matrix projects points in the camera frame. The point is generated
			//in the real world so it need to be translated to the leftcameraframe.
			Point3dimensions_left <<  cloud.at(i).x , cloud.at(i).y, cloud.at(i).z, 1;
			
			/*Project the 3D point onto the Left camera plane [u v w]_left = ProjMat * [X Y Z 1]'
			  Matrix Dimensions:[3x1] = [3x4] * [1x4]'.                                       */
			Point2dimensions_left = A*Point3dimensions_left.transpose();
			
			
			//Divide by the Homogeneous coordinate to obtain the [X_left, Y_left]
			aux_l.x = Point2dimensions_left(0)/Point2dimensions_left(2);
			aux_l.y = Point2dimensions_left(1)/Point2dimensions_left(2);

			Point2dimensions_left(0) = aux_l.x;
			Point2dimensions_left(1) = aux_l.y;
			Point2dimensions_left(2) = 1;

			/*Transform the point from the left camera view to the right camera view
				  using the Homography matrix : [u v w]_right = H*[u v w]_left.
				  Matrix Dimensions: [3x1] = [3x3] * [3x1]   	    */

			Point2dimensions_right = H*Point2dimensions_left; 
			//std::cout << Point2dimensions_right << std::endl;

			//Divide by the Homogeneous coordinate to obtain the [X_right, Y_right]
			aux_r.x = Point2dimensions_right(0)/Point2dimensions_right(2);
			aux_r.y = Point2dimensions_right(1)/Point2dimensions_right(2);


			/*						std::cout << "Right: " << aux_r.x << "  " << aux_r.y <<std::endl;
				std::cout << "Left: " << aux_l.x << "  " << aux_l.y <<std::endl;*/

			if(aux_l.x < 140 && aux_l.x >-140 && aux_r.x < 140 && aux_r.x > -140 && aux_l.y < 110 && aux_l.y > -110 && aux_r.y < 110 && aux_r.y >-110 ){
				modelpoints2dleft.push_back(aux_l);
				modelpoints2dright.push_back(aux_r);
			}else {							

				//std::cout << aux_l.x << std::endl;
				/*					std::cout << aux_l << std::endl;
					std::cout << aux_r << std::endl;*/
				//std::cout << "--------" << std::endl;
			}
		}
	}else std::cerr << "ERROR: Can not find Homography" <<std::endl;

}

