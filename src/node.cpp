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
//PCL
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_types.h>
//EIGEN
#include <Eigen/Eigen>

//Leap Motion SDK
#include "Leap.h"

//OPENCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
//Catkin
#include <leap_object_tracking/feature_detection.h>
#include <leap_object_tracking/leap_camera.h>
#include <leap_object_tracking/disparity.h>


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
const float f = 69.0143432617;
const float T = 0.0401673726737;

void ImagesCallback(const ImageConstPtr& imageLeft, const ImageConstPtr& imageRight){
	
	cv_bridge::CvImageConstPtr bridgeLeft;
	cv_bridge::CvImageConstPtr bridgeRight;

	
	
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
	

	
	detectFeatures detector(bridgeLeft->image,bridgeRight->image);
	
	//Keypoints Detection
	detector.FAST_Detector();
	
	//Keypoints Extractor
	detector.ORB_Extractor();
	
	//Keypoints Matching from left and right images
	detector.BruteForce_Matcher();
	
	Disparity disp(bridgeLeft->image,bridgeRight->image);
	disp.computeDisparity();
	disp.show_disparity();
	
	pcl::PointCloud<pcl::PointXYZ> cloud;
	
	cloud.width = detector.GetGoodMatches().size();	
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.points.resize(cloud.width * cloud.height);
	
	for(size_t i = 0; i < cloud.points.size(); ++i){
				
		int x; int y;
		
		x = detector.GetLeftKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.x;
		y = detector.GetLeftKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.y;
				
		cloud.points[i].x = detector.GetLeftKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.x;
		cloud.points[i].y = detector.GetLeftKeyPoints()[detector.GetGoodMatches()[i].queryIdx].pt.y;
		cloud.points[i].z = (f*T)/disp.Get_DisparityIMG().at<float>(x,y);
	}
	
	//Show Images and drawed features
	detector.Show_LeftCam();
	detector.Show_RightCam();
	
	detector.Draw_Keypoints();
	detector.Draw_Matches();
	detector.Draw_GoodMatches();

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
	
	
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageLeft_sub, imageRight_sub);
	sync.registerCallback(boost::bind(&ImagesCallback, _1, _2));
	

	ros::spin();
  
	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}
