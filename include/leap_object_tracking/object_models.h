#ifndef OBJECT_MODELS_H
#define OBJECT_MODELS_H

///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <iostream>
#include "ros/ros.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "pcl_ros/point_cloud.h" 
#include <leap_object_tracking/particle.h>


class Models{

private:

	std::vector<cv::Point3f> ModelPoints;
	pcl::PointCloud<pcl::PointXYZ> ModelCloud;

public:

	//Constructor and Destructor
	Models(){}
	~Models(){}

	//Get and Set Methods
	void Set_PointCloud(pcl::PointCloud<pcl::PointXYZ> cloud){ ModelCloud = cloud;}
	pcl::PointCloud<pcl::PointXYZ> Get_PointCloud(){return ModelCloud;}
	std::vector<cv::Point3f> Get_ModelPoints(){return ModelPoints;}

	//Methods to construct and transform the 3D objects
	void Cube(double, cv::Point3f);
	void Cylinder(Particle, float, float);
	void GenerateModelCloud();
	cv::Point3f Transform(cv::Point3f, Particle);
};

#endif