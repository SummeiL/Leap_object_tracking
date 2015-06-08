#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <iostream> 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <leap_object_tracking/camera_frames.h>
#include <leap_object_tracking/object_models.h>
#include <leap_object_tracking/particle.h>

//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////


class ParticleFilter{

private:
	
	std::vector<cv::Point2d> ModelInCameraPlane; 
	std::vector<Particle> FilterParticles;
	std::vector<Particle> FilterParticlesWithCovariance;
	int nparticles;	//Particles of the filter
	double M; //Number of samples to draw
	Models model;
	
	//Gaussian distribution parameters
	int size; // Dimensionality (rows)
	pcl::PointCloud<pcl::PointXYZ> FilterCloud;
		
public:
	
	//Constructor and Destructor
	explicit ParticleFilter(): size(6){}
	~ParticleFilter(){}
	
	//SetMethods
	void Set_nparticles(int nparticles){ this->nparticles = nparticles;}
	void Set_ParticlestoDraw(double M){ this->M = M;}
	//Get Mehthods
	pcl::PointCloud<pcl::PointXYZ> GetFilterCloud(){return FilterCloud;}
	
	//Particle Filter Steps and functions
	void InitializePF();
	void MotionModel();
	void MeasurementModel_A(CameraFrames);
	void Resampling();
	void Statistics();
	void DrawParticles(CameraFrames);
	void CloudParticles();
	
	Eigen::MatrixXd MultivariateGaussian(float, float, float, float, float, float,int);
};

#endif
