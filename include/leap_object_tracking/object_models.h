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
	explicit Models(){}
	~Models(){}

	//Get and Set Methods
	void Set_PointCloud(pcl::PointCloud<pcl::PointXYZ> cloud){ ModelCloud = cloud;}
	pcl::PointCloud<pcl::PointXYZ> Get_PointCloud(){return ModelCloud;}


	//Methods to construct and transform the 3D objects
	void Cube(float);
	void Cylinder(Particle, float, float);
	void Point();
	void GenerateModelCloud();
	Eigen::MatrixXf Transform(Particle);
};

#endif