    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"

//Leap Motion SDK
#include "/home/juanma/LeapSDK/include/Leap.h"

//OPENCV
#include <opencv2/imgproc/imgproc.hpp>

//Catkin
#include <leap_object_tracking/feature_detection.h>
#include <leap_object_tracking/leap_camera.h>

    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

	ros::init(argc, argv, "node");
	
	//Create a sample listener and controller
	CameraListener listener;
	Controller controller;
	controller.setPolicy(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_OPTIMIZE_HMD));

	// Have the sample listener receive events from the controller
	controller.addListener(listener);
	controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
  
	//Feature Detection
	Features p;

	ros::spin();
  
	// Remove the sample listener when done
	controller.removeListener(listener);

	return 0;
}
