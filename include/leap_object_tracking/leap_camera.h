// Inclusion guard to prevent this header from being included multiple times
#ifndef LEAP_CAMERA_H
#define LEAP_CAMERA_H


using namespace Leap;

class CameraListener : public Listener {
  public:
  //ros::NodeHandle _node;
  boost::shared_ptr <ros::NodeHandle> _left_node;
  boost::shared_ptr <ros::NodeHandle> _right_node;
  
  ros::Publisher _pub_image_left;
  ros::Publisher _pub_info_left;
  ros::Publisher _pub_image_right;
  ros::Publisher _pub_info_right;

  
  camera_info_manager::CameraInfoManager* info_mgr_right;
  camera_info_manager::CameraInfoManager* info_mgr_left;
  unsigned int seq;
  virtual void onInit(const Controller&);
  virtual void onConnect(const Controller&);
  virtual void onDisconnect(const Controller&);
  virtual void onExit(const Controller&);
  virtual void onFrame(const Controller&);
  virtual void onFocusGained(const Controller&);
  virtual void onFocusLost(const Controller&);
  virtual void onDeviceChange(const Controller&);
  virtual void onServiceConnect(const Controller&);
  virtual void onServiceDisconnect(const Controller&);  
  private:
};


#endif
