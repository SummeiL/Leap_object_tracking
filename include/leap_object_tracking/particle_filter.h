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
#include <leap_object_tracking/stereo_camera.h>
#include <leap_object_tracking/object_models.h>
#include <leap_object_tracking/particle.h>

//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////


class ParticleFilter{

private:
	
	CameraFrames NewFrame;
	StereoCamera NewcamModel;
	
	std::vector<cv::Point2d> ModelInCameraPlane;
	std::vector<Particle> FilterParticles;
	std::vector<std::vector<Particle> > FilterParticlesWithCovariance;
	int nparticles;
	
	//Gaussian distribution parameters
	int nn;     // How many samples (columns) to draw
	int size; // Dimensionality (rows)
		
public:
	
	//Constructor and Destructor
	explicit ParticleFilter(): nn(5), size(6){}
	~ParticleFilter(){}
	
	//SetMethods
	void Set_nparticles(int nparticles){ this->nparticles = nparticles;}
	void Set_NewFrame(CameraFrames NewFrame){this->NewFrame = NewFrame;}
	void Set_NewCamModel(StereoCamera NewcamModel){this->NewcamModel = NewcamModel;}
	
	//Particle Filter Steps and functions
	void InitializePF();
	void MotionModel();
	void MeasurementModel();
	void Resampling();
	
	Eigen::MatrixXd MultivariateGaussian(float, float, float, float, float, float);
};

#endif
