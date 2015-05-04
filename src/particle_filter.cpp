    ///////////////////////////////////////////////////////////////////
   ////                                                           ////
  ////                          INCLUDES                         ////
 ////                                                           ////
///////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <iostream> 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <leap_object_tracking/particle_filter.h>
#include <cmath>


    //////////////////////////////////////////////////////////////////
   ////                                                          ////
  ////                            CODE                          ////
 ////                                                         ////
//////////////////////////////////////////////////////////////////


using namespace std;

ParticleFilter::ParticleFilter(CameraFrames ActualFrame, CameraFrames OldFrame, int nparticles = 500){
	
	this->ActualFrame = ActualFrame;
	this->OldFrame = OldFrame;
	this->nparticles = nparticles;
	

}

void ParticleFilter::GeneratePFcloud(){
	
		PFcloud.header.frame_id = "leap_optical_frame";
		PFcloud.height = 1;
		PFcloud.width = nparticles;
		PFcloud.points.resize (PFcloud.width * PFcloud.height);
		PFcloud.is_dense = false; 
	
}

void ParticleFilter::InitializePF(StereoCamera C){

	GeneratePFcloud();

	srand(time(NULL));

	float x_rand;
	float y_rand;
	float z_rand;
	float x_low, y_low, x_high, y_high;


	Point2d leftTop(105,80);
	Point2d rightBot(175,140);
	Point3d leftTop3d;
	Point3d rightBot3d;

	leftTop3d = C.projectOnepointTo3d(leftTop, C.GetdispFromZ(0.05));
	rightBot3d = C.projectOnepointTo3d(rightBot, C.GetdispFromZ(0.05));
	
	x_low = leftTop3d.x;
	x_high = rightBot3d.x;
	
	y_low = leftTop3d.y;
	y_high = rightBot3d.y;

	if(x_low > rightBot3d.x){
		x_low = rightBot3d.x;
		x_high = leftTop3d.x;
	}

	if(y_low > rightBot3d.y){
		y_low = rightBot3d.y;
		y_high = leftTop3d.y;
	}
	
	//cout << "x_low = " << x_low << "   x_high = " << x_high << "   y_low = "<< y_low << "   y_high = " << y_high << endl;

	for (unsigned int i = 0; i < PFcloud.size(); ++i)
	{
		//Generate random number for the limits  value = rand() % (high-low) + low

		x_rand = (x_low) + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(x_high-(x_low)))); //Between [-2.86863, 2.7548]m
		y_rand = (y_high) + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(y_high-(y_low))));  //Between [-2.1049,2.3135]m
		z_rand = 0.05;

		//Distribute the Points randomly in that region
		PFcloud.points[i].x = x_rand;
		PFcloud.points[i].y = y_rand;
		PFcloud.points[i].z = z_rand;
	}

}

void ParticleFilter::ComputeDiffs(vector<cv::Point3d> Actual3dPoints, vector<cv::Point3d> Old3dPoints){


	if(Actual3dPoints.size() == 0) { cout << "zero" << endl;}

	double aux2_diffX = 0; double aux2_diffY = 0; double aux2_diffZ = 0;

	for(int i = 0; i < Actual3dPoints.size(); i++){

		double diffX = 0; double diffY = 0; double diffZ = 0;
		double aux_diffX = 0; double aux_diffY = 0; double aux_diffZ = 0;

		for(int j = 0; j < Old3dPoints.size(); j++){

			aux_diffX = Actual3dPoints.at(i).x - Old3dPoints.at(j).x;
			aux_diffY = Actual3dPoints.at(i).y - Old3dPoints.at(j).y;
			aux_diffZ = Actual3dPoints.at(i).z - Old3dPoints.at(j).z;

			diffX = diffX + aux_diffX;
			diffY = diffY + aux_diffY;
			diffZ = diffZ + aux_diffZ;
		}

		aux2_diffX = aux2_diffX + (diffX/Old3dPoints.size());
		aux2_diffY = aux2_diffY + (diffY/Old3dPoints.size());
		aux2_diffZ = aux2_diffZ + (diffZ/Old3dPoints.size());		
	}

	if(Actual3dPoints.size() == 0 || Old3dPoints.size() == 0){

		deltaX = 0;
		deltaY = 0;
		deltaZ = 0;

	}else{

		deltaX = aux2_diffX/(Actual3dPoints.size());
		deltaY = aux2_diffY/(Actual3dPoints.size());
		deltaZ = aux2_diffZ/(Actual3dPoints.size());
	}

}

double ParticleFilter::sample(double b_square){

	double b = sqrt(b_square);
	double sum = 0;

	for(int i = 0; i < 12; i++){
		
		double r = ((float) rand())/(float)RAND_MAX;
		double c = 2*b*r;
		sum = sum + c - b;
	}

	return(sum/2);
}

void ParticleFilter::MotionModel(StereoCamera camModelNew, StereoCamera camModelOld){
	
	variance = 0.3;
	std::vector <Point2d> Actual2dKeypoints;
	std::vector <Point2d> Old2dKeypoints;
	
	detectFeatures detect(ActualFrame, OldFrame);
	detect.FAST_Detector();
	detect.ORB_Extractor();
	detect.BruteForce_Matcher();
	detect.Draw_GoodMatches();
	
	for(int i = 0; i < detect.GetGoodMatches().size(); i++){
		
		Actual2dKeypoints.push_back(detect.GetFirstKeyPoints()[detect.GetGoodMatches()[i].queryIdx].pt);
		Old2dKeypoints.push_back(detect.GetSecondKeyPoints()[detect.GetGoodMatches()[i].trainIdx].pt);
				
	}
	
	camModelNew.projectpointsTo3d(Actual2dKeypoints);
	camModelOld.projectpointsTo3d(Old2dKeypoints);

	Filter3dPoints(camModelNew, camModelOld, Actual2dKeypoints, Old2dKeypoints);
	
	ComputeDiffs(Actual3dPoints, Old3dPoints);

	if(deltaX != 0 || deltaY != 0 || deltaZ != 0){
			
		for(int i = 0; i < PFcloud.size(); i++){
			
			//Perturb the components adding noise	
			PFcloud.points[i].x = PFcloud.points[i].x + deltaX;
			PFcloud.points[i].y = PFcloud.points[i].y + deltaY;
			PFcloud.points[i].z = PFcloud.points[i].z + deltaZ;
			
		}
	}

}

void ParticleFilter::Filter3dPoints(StereoCamera camModelNew, StereoCamera camModelOld,
		std::vector <Point2d> Actual2dKeypoints, std::vector <Point2d> Old2dKeypoints){
	if(camModelNew.GetPoints3dim().size() != 0){

		for(int i = 0; i < camModelNew.GetPoints3dim().size(); i++){

			if(camModelNew.GetcvDisparity().at<float>(Actual2dKeypoints.at(i).x, Actual2dKeypoints.at(i).y) != -1 
					&& camModelOld.GetcvDisparity().at<float>(Old2dKeypoints.at(i).x, Old2dKeypoints.at(i).y) != -1){

				Actual3dPoints.push_back(camModelNew.GetPoints3dim().at(i));
				Old3dPoints.push_back(camModelOld.GetPoints3dim().at(i));
				
			}					
		}
	}

}






