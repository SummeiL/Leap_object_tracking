///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <leap_object_tracking/object_models.h>


//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////

/*
	Generate the PointCloud of the Object
	The purpose of this functions is just to visualize the object
	Just used to debug purposes
 */

void Models::GenerateModelCloud(){

	ModelCloud.header.frame_id = "world";
	ModelCloud.height = 1;
	ModelCloud.width = ModelPoints.size();
	ModelCloud.points.resize (ModelCloud.width * ModelCloud.height);
	ModelCloud.is_dense = false; 

	for(int i = 0; i < ModelCloud.width ; i++){

		ModelCloud.points[i].x = ModelPoints.at(i).x;
		ModelCloud.points[i].y = ModelPoints.at(i).y;
		ModelCloud.points[i].z = ModelPoints.at(i).z;

	}
	//	cout << ModelCloud.size() << endl;

} 

/*
	Function to rotate + translate points in 3D space
 */

cv::Point3f Models::Transform(cv::Point3f pointmodel, Particle parti){

	Eigen::Matrix3f rotationMatrix;
	Eigen::MatrixXf aux_m(3,1);
	Eigen::MatrixXf aux_m2(1,3);

	//Construction of the Rotation matrix 3x3 with the orientation of the point	
	rotationMatrix =  Eigen::AngleAxisf(parti.GetAlpha(), Eigen::Vector3f::UnitX())
	* Eigen::AngleAxisf(parti.GetBeta(),  Eigen::Vector3f::UnitY())
	* Eigen::AngleAxisf(parti.GetGamma(), Eigen::Vector3f::UnitZ());

	//Construction of the translation vector 3x1 with the pose of the point
	Eigen::MatrixXf translation(3,1);
	translation << parti.GetX(), parti.GetY(), parti.GetZ();


	aux_m << pointmodel.x, pointmodel.y, pointmodel.z; 

	//Compute the transformation = rotation + translation
	// [X', Y', Z'] = RotationMatrix*[X Y Z] + TranslationMatrix
	aux_m2 = (rotationMatrix*aux_m) + translation;

	//Save the point as Point3f and return it
	cv::Point3f transformedpoint;
	transformedpoint.x = aux_m2(0);
	transformedpoint.y = aux_m2(1);
	transformedpoint.z = aux_m2(2);

	return transformedpoint;
}

/*
	Functions to generate the 3D Models
 */

void Models::Cube(double side, cv::Point3f O){
	//To do

}


void Models::Cylinder(Particle p, float radius, float heigth){

	float step_a= (2*M_PI)/72;
	float step_h = heigth/200;

	for(float h = 0; h < heigth; h+=step_h)
		
		for(float i = 0; i < 2*M_PI  ; i+=step_a){

			cv::Point3f aux(0,0,0);

			if(h == 0 || (h < heigth + 0.001 && h > heigth - 0.001)){
				
				aux.x = radius*cos(i);
				aux.y = radius*sin(i);
				aux.z = h;
				aux = Transform(aux,p);
				
				ModelPoints.push_back(aux);
			}
			if((sqrt(pow(radius*cos(i),2)+pow(radius*sin(i),2)) < radius+0.0005 
					&& sqrt(pow(radius*cos(i),2)+pow(radius*sin(i),2)) > radius-0.0005 ) 
					&& h != 0 && !(h < heigth + 0.01 && h > heigth + 0.01)){
				
				aux.x = radius*cos(i);
				aux.y = radius*sin(i);
				aux.z = h;
				aux = Transform(aux,p);
				
				ModelPoints.push_back(aux);
			}
		}
}