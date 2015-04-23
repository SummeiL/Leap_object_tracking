    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////

//ROS and System
#include <stdio.h>
#include <iostream> 
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

//OPENCV
#include <highgui.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/highgui/highgui.hpp"

//Catking
#include <leap_object_tracking/point3D.h>


    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

Point3D::Point3D(){
	
	x = 0;
	y = 0;
	z = 0;
	
}

Point3D::Point3D(float x_p, float y_p, float z_p){
	
	x = x_p;
	y = y_p;
	z = z_p;
	
}

void Point3D::SetX(float x_p){ x = x_p;}

void Point3D::SetY(float y_p){y = y_p;}

void Point3D::SetZ(float z_p){z = z_p;}

float Point3D::GetX(){return x;}

float Point3D::GetY(){return y;}

float Point3D::GetZ(){return z;}


	
