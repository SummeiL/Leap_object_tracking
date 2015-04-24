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
 #include "opencv2/calib3d/calib3d.hpp"
//Catking
#include <leap_object_tracking/disparity.h>


    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                          ////
//////////////////////////////////////////////////////////////////

	Disparity::Disparity(Mat left_rect, Mat right_rect){
		
		left = left_rect;
		right = right_rect;	
		
	}

	
	void Disparity::computeDisparity(){
		
		stereo(left, right, disparity_img, CV_32F);
		
	}
	Mat Disparity::Get_DisparityIMG(){
		return disparity_img;
	}
	void Disparity::show_disparity(){
		
		Mat disp_show;
		double minVal; double maxVal;
		
		minMaxLoc(disparity_img, &minVal, &maxVal);
		
		disparity_img.convertTo(disp_show, CV_8U, 255/(maxVal-minVal));
		imshow("Disparity", disp_show);
		
	}
