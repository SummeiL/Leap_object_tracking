    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <iostream> 
#include <ros/ros.h>
#include <leap_object_tracking/particle_filter.h>


    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                         ////
//////////////////////////////////////////////////////////////////


using namespace std;

void ParticleFilter::GeneratePFcloud(){
	
		PFcloud.header.frame_id = "leap_optical_frame";
		PFcloud.height = 1;
		PFcloud.width = nparticles;
		PFcloud.points.resize (PFcloud.width * PFcloud.height);
		PFcloud.is_dense = false; 
	
}

void ParticleFilter::InitializePF(){
	
	GeneratePFcloud();
	
	srand(time(NULL));
	
	float x_rand;
	float y_rand;
	float z_rand;

		for (unsigned int i = 0; i < PFcloud.size(); ++i)
		{
			//Generate random number for the limits  value = rand() % (high-low) + low
			
			x_rand = (-2.86863) + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2.7548-(-2.86863)))); //Between [-2.86863, 2.7548]m
			y_rand = (-2.1049) + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2.3135-(-2.1049))));  //Between [-2.1049,2.3135]m
			z_rand = (0.01) + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(0.03-0.01))); //Between [0.01, 0.03]m
			
			//Distribute the Points randomly in that region
			PFcloud.points[i].x = x_rand;
			PFcloud.points[i].y = y_rand;
			PFcloud.points[i].z = z_rand;
		}
	
}
