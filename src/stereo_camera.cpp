///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////


#include <leap_object_tracking/stereo_camera.h>


//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////


StereoCamera::StereoCamera(const CameraFrames& camFrames){

	Frames = camFrames;
	

}

StereoCamera::StereoCamera(const StereoCamera &f){

	*this = f;

}

StereoCamera& StereoCamera::operator = (const StereoCamera &f){

	if(this!=&f){

		this->Frames = f.Frames;
		this->camModel = f.camModel;
		this->modelpoints2dleft = f.modelpoints2dleft;
		this->modelpoints2dright = f.modelpoints2dright;

	}	
}

void StereoCamera::Camera_Model(){

	camModel.fromCameraInfo(Frames.GetLeftInfo(),Frames.GetRightInfo());

}

/*
	Computes the Homography matrix with keypoint detectors
 */

void StereoCamera::FindHomography(){

	std::vector<cv::Point2f> left;
	std::vector<cv::Point2f> right;

	detectFeatures detect(Frames);

	//Detect Keypoints in both images left/right
	detect.FAST_Detector();

	//Extract Features from the keypoints left/right
	detect.SURF_Extractor();

	//Match the keypoints of the two images
	detect.BruteForce_Matcher();

	//std::cout<< detect.GetGoodMatches().size() <<std::endl;

	for(int i = 0; i < detect.GetGoodMatches().size(); i++){

		left.push_back(detect.GetMatchedPoint(i, 1));
		right.push_back(detect.GetMatchedPoint(i, 2));		

	}
	
	detect.Draw_GoodMatches();

	//findHomography needs at least 4 points to comput it, and almost 10 to be accurate
	if(left.size() < 10 || right.size() < 10){

		H.setZero();

	}else{
		
		//Compute the Homography image between left and right images
		cv::Mat_<float> H_aux;
		H_aux = cv::findHomography(left, right, CV_RANSAC);
		
		//Transform it to Eigen Matrix
		cv::cv2eigen(H_aux, H);
	}
}


/*
	Projects the points in the left and right plane of the camera
 */

void StereoCamera::ProjectToCameraPlane(std::vector<cv::Point3f> cloud){

	Eigen::MatrixXf A(3,4);
	Eigen::MatrixXf Point2dimensions_right(1,3);
	Eigen::MatrixXf Point3dimensions_left(1,4);
	Eigen::MatrixXf Point2dimensions_left(1,3);
	
	modelpoints2dright.clear();
	modelpoints2dleft.clear();

	//Construct the Projection Matrix of the Left Camera
	A(0,0) = Frames.GetLeftInfo()->P[0];
	A(0,1) = Frames.GetLeftInfo()->P[1];
	A(0,2) = Frames.GetLeftInfo()->P[2];
	A(0,3) = Frames.GetLeftInfo()->P[3];
	A(1,0) = Frames.GetLeftInfo()->P[4];
	A(1,1) = Frames.GetLeftInfo()->P[5];
	A(1,2) = Frames.GetLeftInfo()->P[6];
	A(1,3) = Frames.GetLeftInfo()->P[7];
	A(2,0) = Frames.GetLeftInfo()->P[8];
	A(2,1) = Frames.GetLeftInfo()->P[9];
	A(2,2) = Frames.GetLeftInfo()->P[10];
	A(2,3) = Frames.GetLeftInfo()->P[11];

	//Find the Homography Matrix that relates left to right camera plane
	
	FindHomography();
	
	if (!H.isZero(0.000000)){
		for(int i = 0; i < cloud.size(); i++){
			cv::Point2f aux_l, aux_r;

			//Store the 3D point in a vector and convert it to Homogeneus coordinates [X Y Z 1]
			Point3dimensions_left <<  cloud.at(i).x, cloud.at(i).y, cloud.at(i).z, 1; 

			/*Project the 3D point onto the Left camera plane [u v w]_left = ProjMat * [X Y Z 1]'
			  Matrix Dimensions:[3x1] = [3x4] * [1x4]'.                                       */
			
			Point2dimensions_left = A*Point3dimensions_left.transpose();


			/*Transform the point from the left camera view to the right camera view
			  using the Homography matrix : [u v w]_right = H*[u v w]_left.
			  Matrix Dimensions: [3x1] = [3x3] * [3x1]                               */

			Point2dimensions_right = H*Point2dimensions_left;  

			if(Point2dimensions_left(2) != 0 && Point2dimensions_right(2) != 0){

				//Divide by the Homogeneous coordinate to obtain the [X_left, Y_left]
				aux_l.x = Point2dimensions_left(0)/Point2dimensions_left(2);
				aux_l.y = Point2dimensions_left(1)/Point2dimensions_left(2);

				//Divide by the Homogeneous coordinate to obtain the [X_right, Y_right]
				aux_r.x = Point2dimensions_right(0)/Point2dimensions_right(2);
				aux_r.y = Point2dimensions_right(1)/Point2dimensions_right(2);

				if((aux_l.x < 280 && aux_r.x < 280) && aux_l.y < 220 && aux_r.y < 220 ){
					modelpoints2dleft.push_back(aux_l);
					modelpoints2dright.push_back(aux_r);
				}
			}
		}
	}else std::cerr << "ERROR: Can not find Homography" <<std::endl;

}

