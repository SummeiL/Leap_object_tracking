cmake_minimum_required(VERSION 2.8.3)
project(leap_object_tracking)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs geometry_msgs message_generation sensor_msgs visualization_msgs camera_info_manager rospack camera_calibration_parsers cv_bridge pcl_ros)

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   visualization_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES leap_motion
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime pcl_ros cv_bridge
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  $ENV{LEAP_SDK}/include
  ${roscpp_INCLUDE_DIRS}
)


add_executable(leap_camera src/leap_camera.cpp src/feature_detection.cpp)

target_link_libraries(leap_camera 
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES} opencv_nonfree
  $ENV{LEAP_SDK}/lib/x64/libLeap.so
)


