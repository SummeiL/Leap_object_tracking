    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "camera_info_manager/camera_info_manager.h"

//PCL
#include "pcl_ros/point_cloud.h"

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
#include <leap_object_tracking/stereo_camera.h>
#include <leap_object_tracking/camera_frames.h>
#include <leap_object_tracking/particle_filter.h>

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

ros::Publisher pub_PFCloud;
CameraFrames Old_Frames;
StereoCamera CameraModelOld;
ParticleFilter Filter;
bool a = true;

void ImagesCallback(const ImageConstPtr& imageLeft, 
					const ImageConstPtr& imageRight, 
					const CameraInfoConstPtr& leftInfo, 
					const CameraInfoConstPtr& rightInfo){
	
	
	
	if(a){
		
		Old_Frames = CameraFrames(imageLeft, imageRight, leftInfo, rightInfo);
		CameraModelOld = StereoCamera(Old_Frames);
		CameraModelOld.Camera_Model();
		CameraModelOld.computeDisparity();
		Filter.InitializePF(CameraModelOld);
		a = false;
		pub_PFCloud.publish(Filter.GetPFcloud());
		
	}else{
	
	//Create an object with the images and camera information
	CameraFrames New_Frames(imageLeft, imageRight, leftInfo, rightInfo);
	StereoCamera CameraModelNew(New_Frames);
	CameraModelNew.Camera_Model();
	
	//Compute the Disparity
	CameraModelNew.computeDisparity();
	
	//Particle Filter
	Filter.SetActualFrame(New_Frames);
	Filter.SetOldFrame(Old_Frames);
	Filter.SetnParticles(100);
	
	Filter.MotionModel(CameraModelNew, CameraModelOld);
	
	Old_Frames = New_Frames;
	CameraModelOld = CameraModelNew;
	pub_PFCloud.publish(Filter.GetPFcloud());
	New_Frames.Show_LeftCam();
	}

	
}
	

int main(int argc, char** argv) {

	ros::init(argc, argv, "node");
	ros::NodeHandle nh;
	

	
		//Create a sample listener and controller
		CameraListener listener;
		Controller controller;
		//controller.setPolicy(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_DEFAULT)); 
	
		// Have the sample listener receive events from the controller
		controller.addListener(listener);
		controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));

		pub_PFCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/leap_object_tracking/PFPointCloud",1);
		
		//Aprroximate synchronization of the images from both cameras for the callback
		message_filters::Subscriber<sensor_msgs::Image> imageLeft_sub(nh, "/leap_object_tracking/left/image_raw",1);
		message_filters::Subscriber<sensor_msgs::Image> imageRight_sub(nh, "/leap_object_tracking/right/image_raw",1);
		message_filters::Subscriber<sensor_msgs::CameraInfo>infoLeft_sub(nh, "/leap_object_tracking/left/camera_info",1);
		message_filters::Subscriber<sensor_msgs::CameraInfo>infoRight_sub(nh, "/leap_object_tracking/right/camera_info",1);
		
		typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;
	
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageLeft_sub, imageRight_sub, infoLeft_sub, infoRight_sub);
		sync.registerCallback(boost::bind(&ImagesCallback, _1, _2, _3, _4));
		ros::spin();
  
		// Remove the sample listener when done
		controller.removeListener(listener);
	
	return 0;
}
