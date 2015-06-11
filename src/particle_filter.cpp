///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <leap_object_tracking/particle_filter.h>



//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////


Eigen::VectorXf Mean(6);
Eigen::MatrixXf SampleMatrix(6,1000);
Eigen::MatrixXf SetOfParticles(4,1000);
double sum_w = 0;
static bool first_time;
double r = M_PI/180;
namespace Eigen{
namespace internal {
template<typename Scalar> 
struct scalar_normal_dist_op 
{
	static boost::mt19937 rng;    // The uniform pseudo-random algorithm
	mutable boost::normal_distribution<Scalar> norm;  // The gaussian combinator

	EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

	template<typename Index>
	inline const Scalar operator() (Index, Index = 0) const { return norm(rng); }
};

template<typename Scalar> boost::mt19937 scalar_normal_dist_op<Scalar>::rng;

template<typename Scalar>
struct functor_traits<scalar_normal_dist_op<Scalar> >
{ enum { Cost = 50 * NumTraits<Scalar>::MulCost, PacketAccess = false, IsRepeatable = false }; };
} // end namespace internal
} // end namespace Eigen

std::random_device rd;
std::mt19937 gen(rd());


/*
	Function to initialize the filter with random particles in a fixed space
 */

void ParticleFilter::InitializePF(){
	
	float x_min = -0.025; 		float x_max = -0.026;
	float y_min = 0.06; 		float y_max = 0.065;
	float z_min = 0.03; 		float z_max = 0.04;
	float alpha_min = 0.087;	float alpha_max = 0.104;
	float beta_min = 0; 		float beta_max = 0;
	float gamma_min = 0; 		float gamma_max = 0;

	//Randomly generation in the limits
	std::uniform_real_distribution<float> dis_x(x_min, x_max);
	std::uniform_real_distribution<float> dis_y(y_min, y_max);
	std::uniform_real_distribution<float> dis_z(z_min, z_max);
	std::uniform_real_distribution<float> dis_alpha(alpha_min, alpha_max);
	std::uniform_real_distribution<float> dis_beta(beta_min, beta_max);
	std::uniform_real_distribution<float> dis_gamma(gamma_min, gamma_max);
	
	for (int n = 0; n < nparticles; ++n){
		
		 Particle p(dis_x(gen), dis_y(gen), dis_z(gen), dis_alpha(gen), dis_beta(gen), dis_gamma(gen), n);

		FilterParticles.push_back(p);
		
		SetOfParticles(0,n) = p.GetX();
		SetOfParticles(1,n) = p.GetY();
		SetOfParticles(2,n) = p.GetZ();
		SetOfParticles(3,n) = 1;
		
		 

	}
	//Generate the 3D Model
	model.Cube(0.04);
	//model.Point();
	first_time = true;
}

/*
	Motion Model of the Particle filter
 */


void ParticleFilter::MotionModel(){

	Eigen::MatrixXd samples;
	Particle p;
	//std::cout << " x   y   z   a   b   g" << std::endl;
	for(int i = 0; i < FilterParticles.size(); i++){
		
/*		std::cout << FilterParticles.at(i).GetX() << "   " << FilterParticles.at(i).GetY() << "   " << FilterParticles.at(i).GetZ() << "   " <<
				FilterParticles.at(i).GetAlpha() << "   " << FilterParticles.at(i).GetBeta() << "   " << FilterParticles.at(i).GetGamma() << std::endl;*/
		samples = MultivariateGaussian(FilterParticles.at(i).GetX(),
				FilterParticles.at(i).GetY(),
				FilterParticles.at(i).GetZ(),
				FilterParticles.at(i).GetAlpha(),
				FilterParticles.at(i).GetBeta(),
				FilterParticles.at(i).GetGamma(),100);

		for(int j = 0; j < 100; j++){
	
			p.SetX(samples(0,j)); 
			p.SetY(samples(1,j));
			p.SetZ(fabs(samples(2,j))); //The Z can't be negative (Means behind the camera)
			p.SetAlpha(samples(3,j));
			p.SetBeta(samples(4,j));
			p.SetGamma(samples(5,j));
			

			FilterParticlesWithCovariance.push_back(p);
		}	
	}
	FilterParticles.clear();
}

/*
	Measurement Model: weight particles based on the distance to an edge
 */

void ParticleFilter::MeasurementModel_A(CameraFrames NewFrame){


	std::vector<cv::Point2f> left;
	std::vector<cv::Point2f> right;

	double D_Left, D_Right;
	double D_max = 5;  double D_min = 0;

	double Normalizer;
	double p_Left, p_Right;
	double p_AllPoints;
	double lambda = 0.2; //0.05;
	double a = 0;
	double w = 0.5;
	double counter2 = 0;

	sum_w = 0;

	for(int  i = 0; i < FilterParticlesWithCovariance.size(); i++){

		//Project the 3D Model to the Cameras Plane
		NewFrame.ProjectToCameraPlane(model.Transform(FilterParticlesWithCovariance.at(i)));

		left = NewFrame.GetProjectedModelPointsLeft();
		right = NewFrame.GetProjectedModelPointsRight();

		if(left.size()==0){
			a++;
		}

		p_AllPoints = 0;
		double counter = 0;

		for(int k = 0; k < left.size(); k++){

			if(left.at(k).x > 0 && left.at(k).x < 280 && right.at(k).x > 0 && right.at(k).x < 280
					&& left.at(k).y > 0 && left.at(k).y < 220 && right.at(k).y > 0 && right.at(k).y < 220){

				counter++;

				D_Left = (double) NewFrame.GetLeftDistanceFrame().at<float>((int)left.at(k).x, (int)left.at(k).y);
				D_Right = (double) NewFrame.GetRightDistanceFrame().at<float>((int)right.at(k).x, (int)right.at(k).y);

				if(D_Left >= 0 && D_Left <D_max && D_Right >=0 && D_Right < D_max){
					p_Left = exp(-lambda*D_Left);
					p_Right = exp(-lambda*D_Right);
				}else {
					p_Left = 0;
					p_Right = 0;
				}

				p_AllPoints +=(w*p_Left + w*p_Right);
				//p_AllPoints +=p_Left;

/*													std::cout << "D_Left     D_Right      p_Left      p_Right" << std::endl;
					std::cout << D_Left << "    " << D_Right  << "     " << p_Left  <<"   " << p_Right << std::endl;	*/
			}
		}	

		if(isinf(p_AllPoints/counter) || isnan(p_AllPoints/counter)){

			FilterParticlesWithCovariance.at(i).SetProb(0);

		}else{
			//std::cout << counter<<std::endl;
			//std::cout << "prob_all: " << p_AllPoints << "  counter: " << counter << "  prob= "<< p_AllPoints/counter <<std::endl;
			FilterParticlesWithCovariance.at(i).SetProb(p_AllPoints/counter);
			
			sum_w += p_AllPoints/counter;
		}

		
		left.clear();
		right.clear();
	}
	
	//std::cout << "Not valid particles: " << a << "  from " << FilterParticlesWithCovariance.size() << "particles"<<std::endl;
	
}


void ParticleFilter::Resampling(){
	
	
	float x_mean = 0; float a_mean = 0;
	float y_mean = 0; float b_mean = 0;
	float z_mean = 0; float g_mean = 0;
	double sum_w = 0;
	int q = 0;

	for(int i = 0; i < FilterParticlesWithCovariance.size(); i++){
		
		sum_w += FilterParticlesWithCovariance.at(i).GetProb();
		
	}
	for(int i = 0; i < FilterParticlesWithCovariance.size(); i++){
		FilterParticlesWithCovariance.at(i).SetProb(FilterParticlesWithCovariance.at(i).GetProb()/sum_w);
	}
	
	
	double M = nparticles; //Number of samples to draw
	double r = static_cast<double> (rand())/(static_cast<double>(RAND_MAX)/(1/M));	
	int i = 0;
	double U,c;
	
	
	while(!(FilterParticlesWithCovariance.at(q).GetProb() != 0)){
		q++;
	}
	c = FilterParticlesWithCovariance.at(q).GetProb();
	
	for(double m = 1; m < M; m++){

		U = r + (m-1)*(1/M);

		while(U > c && i < FilterParticlesWithCovariance.size() - 1){

			i  = i + 1;
			c = c + FilterParticlesWithCovariance.at(i).GetProb(); 

		}

/*		std::cout << "Particle chosen: " << i
				<< "   with probability: " << FilterParticlesWithCovariance.at(i).GetProb()<<std::endl;
		*/
		//std::cout << "x: " << FilterParticlesWithCovariance.at(i).GetX() << "     y: " << FilterParticlesWithCovariance.at(i).GetX() << "     z: " << FilterParticlesWithCovariance.at(i).GetZ()<<std::endl; 


		FilterParticles.push_back(FilterParticlesWithCovariance.at(i));
		
		SampleMatrix(0,m)=FilterParticlesWithCovariance.at(i).GetX();
		SampleMatrix(1,m)=FilterParticlesWithCovariance.at(i).GetY();
		SampleMatrix(2,m)=FilterParticlesWithCovariance.at(i).GetZ();
		SampleMatrix(3,m)=FilterParticlesWithCovariance.at(i).GetAlpha();
		SampleMatrix(4,m)=FilterParticlesWithCovariance.at(i).GetBeta();
		SampleMatrix(5,m)=FilterParticlesWithCovariance.at(i).GetGamma();
		
		SetOfParticles(0,m) = FilterParticlesWithCovariance.at(i).GetX();
		SetOfParticles(1,m) = FilterParticlesWithCovariance.at(i).GetY();
		SetOfParticles(2,m) = FilterParticlesWithCovariance.at(i).GetZ();
		SetOfParticles(3,m) = 1;
		
		x_mean +=FilterParticlesWithCovariance.at(i).GetX();
		y_mean +=FilterParticlesWithCovariance.at(i).GetY();
		z_mean +=FilterParticlesWithCovariance.at(i).GetZ();
		a_mean +=FilterParticlesWithCovariance.at(i).GetAlpha();
		b_mean += FilterParticlesWithCovariance.at(i).GetBeta();
		g_mean += FilterParticlesWithCovariance.at(i).GetGamma();
		
		
	}	
	//std::cout << "=================================================" <<std::endl;
	Mean << x_mean/(M-1), y_mean/(M-1), z_mean/(M-1),a_mean/(M-1),b_mean/(M-1),g_mean/(M-1);


	FilterParticlesWithCovariance.clear();
}

void ParticleFilter::Statistics(){

	Eigen::MatrixXf Covariance;
	Eigen::MatrixXf u = Eigen::MatrixXf::Ones(1,M);
	Covariance = (1/M-1)*(SampleMatrix-Mean*u)*(SampleMatrix-Mean*u).transpose();
	
	//std::cout << Mean(0)+0.02 <<  "    " << Mean(1) << "     "<<Mean(2) << std::endl;

}

void ParticleFilter::DrawParticles(CameraFrames NewFrame){

	Eigen::MatrixXf particles(4,nparticles);
	std::vector<cv::Point2f> left;
	std::vector<cv::Point2f> right;
	cv::Scalar a(0,255,0);
	Particle pwm;
	
	cv::Mat leftFrame = NewFrame.GetLeftFrame();
	cv::Mat rightFrame = NewFrame.GetRightFrame();
	
	 cv::Mat img_rgbLeft(leftFrame.size(), CV_8UC3);
	 cv::Mat img_rgbRight(rightFrame.size(), CV_8UC3);
	 cv::cvtColor(leftFrame, img_rgbLeft, CV_GRAY2RGB);	 
	 cv::cvtColor(rightFrame, img_rgbRight, CV_GRAY2RGB);
	 
	 //weighted average
	 Eigen::MatrixXf wm(6,1), tmp(6,1);
	 wm.setZero();
	 double weightsum=0;
	 for(int i=0; i<FilterParticles.size(); i++) {
	    double w = FilterParticles.at(i).GetProb();
	    weightsum+=w;
	    tmp << w*FilterParticles.at(i).GetX(),w*FilterParticles.at(i).GetY(),w*FilterParticles.at(i).GetZ(),w*FilterParticles.at(i).GetAlpha(),w*FilterParticles.at(i).GetBeta(),w*FilterParticles.at(i).GetGamma();
	    wm+=tmp;
	 }
	 wm /= weightsum;
	
	 if(isnan(wm(0))){
		wm.setZero();
	}
	if(wm(0)==0 && wm(1)==0 && wm(2)==0){
		pwm.SetX(FilterParticles.at(0).GetX());
		pwm.SetY(FilterParticles.at(0).GetY());
		pwm.SetZ(FilterParticles.at(0).GetZ());
		pwm.SetAlpha(FilterParticles.at(0).GetAlpha());
		pwm.SetBeta(FilterParticles.at(0).GetBeta());
		pwm.SetGamma(FilterParticles.at(0).GetGamma());
	}else{
		pwm.SetX(wm(0));
		pwm.SetY(wm(1));
		pwm.SetZ(wm(2));
		pwm.SetAlpha(wm(3));
		pwm.SetBeta(wm(4));
		pwm.SetGamma(wm(5));	
	}
	 
	
	 //std::cout << wm(0) << "   " << wm(1) << "   " << wm(2) << "   " << wm(3) << "   "<<wm(4)<<"   "<<wm(5)<<std::endl;
	// Particle p(-0.025,0.07,0.06,5,0,0,0);
	
	//NewFrame.ProjectToCameraPlane(SetOfParticles);
	NewFrame.ProjectToCameraPlane(model.Transform(pwm));
	//NewFrame.ProjectToCameraPlane(model.Transform(p));
	
	left = NewFrame.GetProjectedModelPointsLeft();
	right = NewFrame.GetProjectedModelPointsRight();
	
	for(int k = 0; k < left.size(); k++){
		
		cv::Point center_l(floor(left.at(k).x), floor(left.at(k).y));	
		cv::Point center_r(floor(right.at(k).x), floor(right.at(k).y));
	//NOTE: axes are wrong	
		//cv::Point center_l(floor(left.at(k).y), floor(left.at(k).x));	
		//cv::Point center_r(floor(right.at(k).y), floor(right.at(k).x));


			cv::circle(img_rgbLeft, center_l, 2, a,-1);
			cv::circle(img_rgbRight, center_r, 2, a,-1);

	}
	
	left.clear();
	right.clear();

	cv::imshow("pointsleft", img_rgbLeft);
	cv::imshow("pointsright", img_rgbRight);
	
	cv::waitKey(1);
}

void ParticleFilter::CloudParticles(){
	
	FilterCloud.header.frame_id = "world";
	FilterCloud.height = 1;
	FilterCloud.width = M;
	FilterCloud.points.resize (FilterCloud.width * FilterCloud.height);
	FilterCloud.is_dense = false; 
	
	
	
	for(int i = 0; i < M ; i++){

		FilterCloud.points[i].x = SetOfParticles(0,i);
		FilterCloud.points[i].y = SetOfParticles(1,i);
		FilterCloud.points[i].z = SetOfParticles(2,i);
		

	}
	
}

/*
  Draw nn samples from a size-dimensional normal distribution
  with a specified mean and covariance
 */

Eigen::MatrixXd ParticleFilter::MultivariateGaussian(float x, float y, float z, float a, float b, float c, int nn){

	Eigen::internal::scalar_normal_dist_op<double> randN; // Gaussian functor
	Eigen::internal::scalar_normal_dist_op<double>::rng.seed(1); // Seed the rng

	// Define mean and covariance of the distribution
	Eigen::VectorXd mean(size);       
	Eigen::MatrixXd covar(size,size);

	mean  <<  x, y, z, a, b, c;
	covar <<  .1, 0, 0, 0, 0, 0,
			0, .1, 0, 0, 0, 0,
			0, 0, .01, 0, 0, 0,
			0, 0, 0, .1, 0, 0,
			0, 0, 0, 0, .1, 0,
			0, 0, 0, 0, 0, .1;

	Eigen::MatrixXd normTransform(size,size);

	Eigen::LLT<Eigen::MatrixXd> cholSolver(covar);

	// We can only use the cholesky decomposition if 
	// the covariance matrix is symmetric, pos-definite.
	// But a covariance matrix might be pos-semi-definite.
	// In that case, we'll go to an EigenSolver
	if (cholSolver.info()==Eigen::Success) {
		// Use cholesky solver
		normTransform = cholSolver.matrixL();
	} else {
		// Use eigen solver
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
		normTransform = eigenSolver.eigenvectors() 
	                				   * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
	}

	Eigen::MatrixXd samples = (normTransform 
			* Eigen::MatrixXd::NullaryExpr(size,nn,randN)).colwise() 
	                        				   + mean;

	/*	std::cout << "Mean\n" << mean << std::endl;
	std::cout << "Covar\n" << covar << std::endl;
	std::cout << "Samples\n" << samples << std::endl;
	cout << "------------" << endl;*/

	return(samples);
}

