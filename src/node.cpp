    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <iostream>
#include <string.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "ros/ros.h"
#include "camera_info_manager/camera_info_manager.h"
#include "stereo_msgs/DisparityImage.h"
#include <image_transport/image_transport.h>

//Leap Motion SDK
#include "/home/juanma/LeapSDK/include/Leap.h"

//OPENCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
//Catkin
#include <leap_object_tracking/feature_detection.h>
#include <leap_object_tracking/leap_camera.h>
#include <leap_object_tracking/point3D.h>

    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace stereo_msgs;

void Print3DPoints(std::vector<Point3D> Points);


void ImagesCallback(const ImageConstPtr& imageLeft, const ImageConstPtr& imageRight, const stereo_msgs::DisparityImageConstPtr& disparity){
	
	cv_bridge::CvImageConstPtr bridgeLeft;
	cv_bridge::CvImageConstPtr bridgeRight;
	cv_bridge::CvImageConstPtr bridgeDisparity;
	
	std::vector<Point3D> Points;
	
	try{
		
		bridgeLeft = cv_bridge::toCvCopy(imageLeft, "mono8");
		
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Left ros image.");
		return;
	}
	
	try{
		
		bridgeRight = cv_bridge::toCvCopy(imageRight, "mono8");
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Right ros image.");
		return;
	}
	
	try{
		
		bridgeDisparity = cv_bridge::toCvShare(disparity->image, disparity, "32FC1");
		
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Depth ros image.");
		return;
	}
	
	imshow("Disparity", bridgeDisparity->image);
	
		Mat outDis;
	outDis = bridgeDisparity->image;
	
	detectFeatures detector(bridgeLeft->image,bridgeRight->image);
	
	//Keypoints Detection
	detector.FAST_Detector();
	
	//Keypoints Extractor
	detector.ORB_Extractor();
	
	//Keypoints Matching from left and right images
	detector.BruteForce_Matcher();
	
	//Extraction of the XY position from matchings to generate 3DPoints.
	for(int i = 0; i < detector.GetGoodMatches().size(); i++){
		
		Point3D point;
		float depth;
		
		point.SetX(detector.GetLeftKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.x);
		point.SetY(detector.GetRightKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.y);	
		
		if (outDis.at<float>(point.GetX(), point.GetY()) == -1){
			
			depth = 0;
			
		}else{
			
			depth = ((disparity->f)*(disparity->T))/(outDis.at<float>(point.GetX(), point.GetY()));
			
		}
		
		point.SetZ(depth);
		Points.push_back(point);
		
	}
	

	Print3DPoints(Points);
	
	//Show Images and drawed features
	detector.Show_LeftCam();
	detector.Show_RightCam();
	imshow("Disparity", bridgeDisparity->image);
	
	detector.Draw_Keypoints();
	detector.Draw_Matches();
	detector.Draw_GoodMatches();
	

	
	
}


/*void disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity){
	

	cv_bridge::CvImageConstPtr bridge;
	
	
	try{
		
		bridge = cv_bridge::toCvShare(disparity->image, disparity, "32FC1");
		
	}
	catch (cv_bridge::Exception& e){
		
		ROS_ERROR("Failed to transform Depth ros image.");
		return;
	}
	
	cv::Mat  out;
	out = bridge->image;
	
	ROS_INFO("valor: %f", out.at<float>(0,0));
	
	imshow("Disparity", bridge->image);
	

	}*/

void Print3DPoints(std::vector<Point3D> Points){
	
	cout << "Number of 3DPoints: " << Points.size() << endl;
	
	
	for(int i = 0; i < Points.size(); i++){
		
		cout << "X: " << Points[i].GetX() << "    Y:  " << Points[i].GetY() << "     Z:  " << Points[i].GetZ() << endl;

	}
	
}
	

int main(int argc, char** argv) {

	ros::init(argc, argv, "node");
	ros::NodeHandle nh;
	
	//Create a sample listener and controller
	CameraListener listener;
	Controller controller;
	controller.setPolicy(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_OPTIMIZE_HMD));

	// Have the sample listener receive events from the controller
	controller.addListener(listener);
	controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
	
	//ros::Subscriber disparity_sub = nh.subscribe("/leap_object_tracking/disparity",1,disparityCallback);
	
	//Aprroximate synchronization of the images from both cameras for the callback
	message_filters::Subscriber<sensor_msgs::Image> imageLeft_sub(nh, "/leap_object_tracking/left/image_raw",1);
	message_filters::Subscriber<sensor_msgs::Image> imageRight_sub(nh, "/leap_object_tracking/right/image_raw",1);
	message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub(nh, "/leap_object_tracking/disparity",1);
	
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, stereo_msgs::DisparityImage> MySyncPolicy;
	
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageLeft_sub, imageRight_sub, disparity_sub);
	sync.registerCallback(boost::bind(&ImagesCallback, _1, _2,_3));
	

	ros::spin();
  
	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}
