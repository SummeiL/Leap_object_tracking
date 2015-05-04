#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "pcl_ros/point_cloud.h"
#include <leap_object_tracking/camera_frames.h>
#include <leap_object_tracking/feature_detection.h>
#include <leap_object_tracking/stereo_camera.h>

using namespace std;


class ParticleFilter{
	
	private:
	
	pcl::PointCloud<pcl::PointXYZ> PFcloud;
	int nparticles;
	double deltaX, deltaY, deltaZ;
	double variance;
	CameraFrames ActualFrame, OldFrame;
	vector<cv::Point3d> Actual3dPoints;
	vector<cv::Point3d> Old3dPoints;
	
	public:
	
	ParticleFilter(){ nparticles = 100;}
	ParticleFilter(CameraFrames, CameraFrames, int);
	
	void SetActualFrame(CameraFrames ActualFrame){ this->ActualFrame = ActualFrame;}
	void SetOldFrame(CameraFrames OldFrame){ this->OldFrame = OldFrame;}
	void SetnParticles(int nparticles){ this->nparticles = nparticles;}
	void GeneratePFcloud();
	void InitializePF(StereoCamera);
	void ComputeDiffs(vector<cv::Point3d>, vector<cv::Point3d>);
	double sample(double);
	void MotionModel(StereoCamera, StereoCamera);
	void MeasurementModel();
	void Resampling();
	pcl::PointCloud<pcl::PointXYZ> GetPFcloud(){return PFcloud;}
	void Filter3dPoints(StereoCamera, StereoCamera, std::vector <Point2d>, std::vector <Point2d>);
	
	
};

#endif
