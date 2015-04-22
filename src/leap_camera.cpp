    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"
#include "rospack/rospack.h"

//Leap Motion SDK
#include "/home/juanma/LeapSDK/include/Leap.h"

//Catkin
#include <leap_object_tracking/leap_camera.h>

    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

#define targetWidth 500
#define targetHeight 500
#define cutWidth 280
#define cutHeight 220
#define startX 110
#define startY 140

using namespace Leap;
using namespace std;

void CameraListener::onInit(const Controller& controller){


  std::string path;
  std::vector<std::string> search_path;
  std::string default_l_info_filename;
  std::string default_r_info_filename;
  
  _left_node = boost::make_shared<ros::NodeHandle>("/leap_object_tracking/left");
  _right_node = boost::make_shared<ros::NodeHandle>("/leap_object_tracking/right");
  
  std::cout << "Initialized" << std::endl;
  
  _pub_image_left = _left_node->advertise<sensor_msgs::Image>("image_raw", 1);
  _pub_info_left = _left_node->advertise<sensor_msgs::CameraInfo>("camera_info", 1);
  _pub_image_right = _right_node->advertise<sensor_msgs::Image>("image_raw", 1);
  _pub_info_right = _right_node->advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  seq = 0;

  
  rospack::Rospack rp;
  
  rp.getSearchPathFromEnv(search_path);
  rp.crawl(search_path, 1);
  
  if (rp.find("leap_object_tracking",path)==true){
	  
    default_l_info_filename = path + std::string("/camera_info/leap_cal_left.yml");
    default_r_info_filename = path + std::string("/camera_info/leap_cal_right.yml");
    
  }
  else {
	  
    default_l_info_filename = "";
    default_r_info_filename = "";
    
  }

  ros::NodeHandle local_nh("~");
  
  std::string l_info_filename;
  std::string r_info_filename;
  
  local_nh.param("template_filename_left", l_info_filename, default_l_info_filename);
  local_nh.param("template_filename_right", r_info_filename, default_r_info_filename);
  
  l_info_filename = std::string("file://") + l_info_filename;
  r_info_filename = std::string("file://") + r_info_filename;
  
  info_mgr_left = new camera_info_manager::CameraInfoManager(*_left_node, "left", l_info_filename);
  info_mgr_right = new camera_info_manager::CameraInfoManager(*_right_node, "right", r_info_filename);
}

void CameraListener::onConnect(const Controller& controller) {
  
  std::cout << "Connected" << std::endl;

}

void CameraListener::onDisconnect(const Controller& controller) {
  
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
  
}

void CameraListener::onExit(const Controller& controller) {
  
  std::cout << "Exited" << std::endl;
  
}

void CameraListener::onFrame(const Controller& controller) {
	
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();  
  ImageList images = frame.images();

  sensor_msgs::Image image_msg;
  image_msg.header.seq = seq++;
  image_msg.header.stamp =ros::Time::now();
  image_msg.encoding ="mono8";
  image_msg.is_bigendian = 0;
  
  for(int camera_num=0; camera_num<2; camera_num++){
    
    Image image = images[camera_num];
    image_msg.width =  cutWidth;
    image_msg.height = cutHeight;
    image_msg.step = cutWidth;
    image_msg.data.resize(cutWidth*cutHeight);
    
	//pragma omp parallel for
	
    for(int i=0; i<cutWidth; i++){
      
      for(int j=0; j<cutHeight; j++){
	
		Vector input = Vector((float)(i+startX)/targetWidth, (float)(j+startY)/targetHeight, 0);
		input.x = (input.x - image.rayOffsetX()) / image.rayScaleX();
		input.y = (input.y - image.rayOffsetY()) / image.rayScaleY();
		Vector pixel = image.warp(input);
		
		if(pixel.x >= 0 && pixel.x < image.width() && pixel.y >= 0 && pixel.y < image.height()) {
			
			int data_index = floor(pixel.y) * image.width() + floor(pixel.x);
			image_msg.data[cutWidth*j+i] = image.data()[data_index];
		
		} else {
			
			image_msg.data[cutWidth*j+i] = 0;
			
		}
      }
    }
    
    if(camera_num==0){
		
      sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_left->getCameraInfo()));
      
      image_msg.header.frame_id = info_msg->header.frame_id ="leap_optical_frame";
      info_msg->width = image_msg.width;
      info_msg->height = image_msg.height;
      info_msg->header.stamp = image_msg.header.stamp;
      info_msg->header.seq = image_msg.header.seq;
      
      _pub_image_left.publish(image_msg);
      _pub_info_left.publish(*info_msg);
      
    }else{
      
      sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_right->getCameraInfo()));
      
      image_msg.header.frame_id = info_msg->header.frame_id = "leap_optical_frame";
      info_msg->width = image_msg.width;
      info_msg->height = image_msg.height;
      info_msg->header.stamp = image_msg.header.stamp;
      info_msg->header.seq = image_msg.header.seq;
     
      _pub_image_right.publish(image_msg);
      _pub_info_right.publish(*info_msg);
      
    }
  }

}

void CameraListener::onFocusGained(const Controller& controller) {
	
  std::cout << "Focus Gained" << std::endl;
  
}

void CameraListener::onFocusLost(const Controller& controller) {
	
  std::cout << "Focus Lost" << std::endl;
  
}

void CameraListener::onDeviceChange(const Controller& controller) {
	
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
	  
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    
  }
}

void CameraListener::onServiceConnect(const Controller& controller) {
	
  std::cout << "Service Connected" << std::endl;
  
}

void CameraListener::onServiceDisconnect(const Controller& controller) {
	
  std::cout << "Service Disconnected" << std::endl;
  
}
