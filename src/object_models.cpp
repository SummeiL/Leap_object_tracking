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

Eigen::MatrixXf Models::Transform(Particle parti){
	
	Eigen::Matrix3f rotationMatrix;
	Eigen::MatrixXf translation(3,1);
	Eigen::MatrixXf points(3,ModelPoints.size());
	Eigen::MatrixXf transformation(3, ModelPoints.size());
	Eigen::MatrixXf u = Eigen::MatrixXf::Ones(1,ModelPoints.size());

	for(int i = 0; i < ModelPoints.size(); i++){
		points(0,i) = ModelPoints.at(i).x;
		points(1,i) = ModelPoints.at(i).y;
		points(2,i) = ModelPoints.at(i).z;		
	}
	
	//Construction of the Rotation matrix 3x3 with the orientation of the point	
	rotationMatrix =  Eigen::AngleAxisf(parti.GetAlpha(), Eigen::Vector3f::UnitX())
					* Eigen::AngleAxisf(parti.GetBeta(),  Eigen::Vector3f::UnitY())
					* Eigen::AngleAxisf(parti.GetGamma(), Eigen::Vector3f::UnitZ());
	
	//Construction of the translation vector 3x1 with the pose of the point
	translation << parti.GetX(), parti.GetY(), parti.GetZ();
	
	transformation = (rotationMatrix*points)+(translation*u);
	
	return transformation;
	
}


/*
	Functions to generate the 3D Models
 */

void Models::Cube(float side){
	
	ModelPoints.clear();
	cv::Point3f aux(0,0,0);

	float step = side/10;
	float dif = 0.000000001;

	for(float i = -side/2; i <= side/2; i+=step){
		for(float j = -side/2; j <= side/2; j+=step){
			for(float k = 0; k <= side+step; k+=step){

				if(k == 0 || k > side+dif ){

					if(j == side/2 || j == -side/2){
						
						aux.x = i;
						aux.y = j;
						aux.z = k;
						ModelPoints.push_back(aux);
					

						aux.x = i;
						aux.y = -j;
						aux.z = k;
						ModelPoints.push_back(aux);

					}

					if(i == side/2 || i == -side/2){

						aux.x = i;
						aux.y = j;
						aux.z = k;
						ModelPoints.push_back(aux);

						aux.x = -i;
						aux.y = j;
						aux.z = k;
						ModelPoints.push_back(aux);
					}
				}else{

					if(i == side/2 && j == side/2){

						aux.x = i;
						aux.y = j;
						aux.z = k;
						ModelPoints.push_back(aux);

						aux.x = -i;
						aux.y = -j;
						aux.z = k;
						ModelPoints.push_back(aux);
					}

					if(i == -side/2 && j == side/2){

						aux.x = i;
						aux.y = j;
						aux.z = k;
						ModelPoints.push_back(aux);

						aux.x = -i;
						aux.y = -j;
						aux.z = k;
						ModelPoints.push_back(aux);


					}

				}


			}
		}
	}
	
}
void Models::Cylinder(Particle p, float radius, float heigth){

	ModelPoints.clear();

	float step_a= (2*M_PI)/18;
	float step_h = heigth/25;
	cv::Point3f aux(0,0,0);

	for(float h = 0; h < heigth; h+=step_h){	
		for(float i = 0; i < 2*M_PI  ; i+=step_a){
			

			if(h == 0 || (h < heigth + 0.001 && h > heigth - 0.001)){

				aux.x = radius*cos(i);
				aux.y = radius*sin(i);
				aux.z = h;

				ModelPoints.push_back(aux);
			}
			if((sqrt(pow(radius*cos(i),2)+pow(radius*sin(i),2)) < radius+0.0005 
					&& sqrt(pow(radius*cos(i),2)+pow(radius*sin(i),2)) > radius-0.0005 ) 
					&& h != 0 && !(h < heigth + 0.01 && h > heigth + 0.01)){

				aux.x = radius*cos(i);
				aux.y = radius*sin(i);
				aux.z = h;
				
				ModelPoints.push_back(aux);
			}
		}
		//std::cout << " points: " <<  ModelPoints.size() << std::endl;
	}
}