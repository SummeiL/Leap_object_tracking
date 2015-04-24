#ifndef DISPARITY_H
#define DISPARITY_H

#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
 #include "opencv2/calib3d/calib3d.hpp"
#include <string.h>
#include <iostream>

using namespace cv;

class Disparity{
	
 private:
	
	Mat left, right;
	Mat disparity_img;
	StereoBM stereo;

 public:
	Disparity(Mat left_rect, Mat right_rect);
	Mat Get_DisparityIMG();
	void show_disparity();
	void computeDisparity();
	
};

#endif
