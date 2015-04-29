#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
#include "pcl_ros/point_cloud.h"
class ParticleFilter{
	
	private:
	
	float x_old, y_old, z_old;
	pcl::PointCloud<pcl::PointXYZ> PFcloud;
	int nparticles;
	
	public:

	ParticleFilter(){nparticles = 100;}
	
	void GeneratePFcloud();
	void InitializePF();
	void MotionModel();
	void MeasurementModel();
	void Resampling();
	pcl::PointCloud<pcl::PointXYZ> GetPFcloud(){return PFcloud;}
	
	
};

#endif
