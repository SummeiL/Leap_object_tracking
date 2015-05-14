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


StereoCamera::StereoCamera(CameraFrames camFrames){

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

	detect.FAST_Detector();
	detect.SURF_Extractor();
	detect.BruteForce_Matcher();

	
	//std::cout<< detect.GetGoodMatches().size() <<std::endl;

	for(int i = 0; i < detect.GetGoodMatches().size(); i++){

		left.push_back(detect.GetMatchedPoint(i, 1));
		right.push_back(detect.GetMatchedPoint(i, 2));		

	}

	if(left.size() < 4 || right.size() < 4){

		std::cerr << "ERROR: Can not find Homography -> Not enough Keypoints detected" <<std::endl;
		

	}else{
		cv::Mat_<float> H_aux;
		H_aux = cv::findHomography(left, right, CV_RANSAC);
		cv::cv2eigen(H_aux, H);	
	}

}


/*
	Projects the points in the left and right plane of the camera
 */

void StereoCamera::ProjectToCameraPlane(std::vector<cv::Point3f> cloud){

	//Obtain Proyection on left plane
	//  [u v w]' = P * [X Y Z 1]'
	//        x_left = u / w
	//        y_left = v / w

	//Obtain proyection on right plane
	// [u v w]  = [u v w]*H
	// 		x_right = u / w
	//		y_right = v / w


	Eigen::MatrixXf A(3,4);
	Eigen::MatrixXf Point2dimensions_right(1,3);
	Eigen::MatrixXf Point3dimensions_left(1,4);
	Eigen::MatrixXf Point2dimensions_left(1,3);

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


	for(int i = 0; i < cloud.size(); i++){

		cv::Point2f aux_l, aux_r;

		//Store the 3D point in a vector [x y z 1]
		Point3dimensions_left <<  cloud.at(i).x, cloud.at(i).y, cloud.at(i).z, 1; 

		//Project the 3D point onto the Left camera plane [u v w]_left' = ProjMat*[X Y Z 1] 
		Point2dimensions_left = A*Point3dimensions_left.transpose();

		//Compute the point on the right camera plane with the Homography 
		//[u v w]_right = [u v w]_left*H 
		Point2dimensions_right = Point2dimensions_left.transpose()*H;  

		if(Point2dimensions_left(2) != 0 && Point2dimensions_right(2) != 0){

			//Divide by the Homogeneous coordinate to obtain the [X_left, Y_left]
			aux_l.x = Point2dimensions_left(0)/Point2dimensions_left(2);
			aux_l.y = Point2dimensions_left(1)/Point2dimensions_left(2);

			//Divide by the Homogeneous coordinate to obtain the [X_right, Y_right]
			aux_r.x = Point2dimensions_right(0)/Point2dimensions_right(2);
			aux_r.y = Point2dimensions_right(1)/Point2dimensions_right(2);
		}

		modelpoints2dleft.push_back(aux_l);
		modelpoints2dright.push_back(aux_r);
	}
}

