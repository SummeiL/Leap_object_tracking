cmake_minimum_required(VERSION 2.8.3)
project(leap_object_tracking)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs geometry_msgs message_generation sensor_msgs visualization_msgs camera_info_manager rospack camera_calibration_parsers cv_bridge pcl_ros message_filters image_geometry stereo_image_proc tf)

find_package(Boost REQUIRED COMPONENTS thread system)
find_package(cmake_modules REQUIRED)
 find_package(Eigen REQUIRED)

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES leap_motion
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime pcl_ros cv_bridge image_geometry stereo_image_proc tf 
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  $ENV{LEAP_SDK}/include
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
${EIGEN_INCLUDE_DIRS}
)

add_executable(node src/node.cpp src/feature_detection.cpp src/object_models.cpp src/leap_camera.cpp src/camera_frames.cpp  src/particle_filter.cpp)
target_link_libraries(node 
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES} opencv_nonfree
  $ENV{LEAP_SDK}/lib/x64/libLeap.so
  ${Boost_LIBRARIES}
)
install(TARGETS node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


