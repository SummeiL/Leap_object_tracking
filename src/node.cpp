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
#include <tf/transform_broadcaster.h>
#include <leap_object_tracking/nodeConfig.h>


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
#include <leap_object_tracking/leap_camera.h>
#include <leap_object_tracking/camera_frames.h>
#include <leap_object_tracking/particle_filter.h>
#include <leap_object_tracking/object_models.h>

//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////

static bool first_time = true;
ros::Publisher pub_PFCloud;
 
ParticleFilter Filter;

/*
	Callback for the Camera Images
*/

void ImagesCallback(const sensor_msgs::ImageConstPtr& imageLeft, 
					const sensor_msgs::ImageConstPtr& imageRight, 
					const sensor_msgs::CameraInfoConstPtr& leftInfo, 
					const sensor_msgs::CameraInfoConstPtr& rightInfo){

	//Frame of the camera = same position as world
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform transformModel;

	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

	transformModel.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transformModel.setRotation( tf::Quaternion(0, 0, 0, 1) );

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "leap_optical_frame"));

	//This will be used for the first iterarion
	if(first_time){ 
		
		//Set Filter Parameters and Initialize it
		Filter.Set_nparticles(100);
		Filter.InitializePF();

		//Change value in order to not repeat these instructions again
		first_time = false;


	}else{
		
		//Adquire frames and apply edge detector
		CameraFrames New_Frames(imageLeft, imageRight, leftInfo, rightInfo);
		New_Frames.EdgeDetector();

		//Shows edge detector output
		New_Frames.Show_LeftDistanceFrame();
		New_Frames.Show_RightDistanceFrame();

		//Motion Model
		Filter.MotionModel();
	
		//Measurement Model
		Filter.MeasurementModel_A(New_Frames);
		
		//Resampling
		Filter.Resampling();
		
		//Compute statistics
		Filter.Statistics();
		
		//Draw Results
		Filter.DrawParticles(New_Frames);
		
	}

	//pub_PFCloud.publish(model.Get_PointCloud());//Uncoment this line for publish the cloud of the model

}



int main(int argc, char** argv) {

	ros::init(argc, argv, "node");
	ros::NodeHandle nh;

/*
	//Create a sample listener and controller
	CameraListener listener;
	Controller controller;
	//controller.setPolicy(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_DEFAULT)); 

	// Have the sample listener receive events from the controller
	controller.addListener(listener);
	controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
*/

	pub_PFCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/leap_object_tracking/PFPointCloud",1);

	//Aprroximate synchronization of the images from both cameras for the callback
	message_filters::Subscriber<sensor_msgs::Image> imageLeft_sub(nh, "/leap_object_tracking/left/image_rect",1);
	message_filters::Subscriber<sensor_msgs::Image> imageRight_sub(nh, "/leap_object_tracking/right/image_rect",1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>infoLeft_sub(nh, "/leap_object_tracking/left/camera_info",1);
	message_filters::Subscriber<sensor_msgs::CameraInfo>infoRight_sub(nh, "/leap_object_tracking/right/camera_info",1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageLeft_sub, imageRight_sub, infoLeft_sub, infoRight_sub);

	sync.registerCallback(boost::bind(&ImagesCallback, _1, _2, _3, _4));
	ros::spin();

	// Remove the sample listener when done
	//controller.removeListener(listener);

	return 0;
}
