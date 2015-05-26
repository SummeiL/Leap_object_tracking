///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <leap_object_tracking/particle_filter.h>
#include <leap_object_tracking/GetTime.h>


//////////////////////////////////////////////////////////////////
////                                                          ////
////                            CODE                          ////
////                                                          ////
//////////////////////////////////////////////////////////////////

//Change to radiants
float r = M_PI/180;

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

	float x_min = 0; 		float x_max = 0.05;
	float y_min = 0; 		float y_max = 0.05;
	float z_min = 0; 		float z_max = 0.05;
	float alpha_min = 0;	float alpha_max = 360;
	float beta_min = 0; 	float beta_max = 360;
	float gamma_min = 0; 	float gamma_max = 360;


	//Randomly generation in the limits x,y,z,a,b,c
	std::uniform_real_distribution<double> dis_x(x_min, x_max);
	std::uniform_real_distribution<double> dis_y(y_min, y_max);
	std::uniform_real_distribution<double> dis_z(z_min, z_max);
	std::uniform_real_distribution<double> dis_alpha(alpha_min, alpha_max);
	std::uniform_real_distribution<double> dis_beta(beta_min, beta_max);
	std::uniform_real_distribution<double> dis_gamma(gamma_min, gamma_max);

	for (int n = 0; n < nparticles; ++n){

		FilterParticles.push_back(Particle(dis_x(gen), dis_y(gen), dis_z(gen), dis_alpha(gen)*r, dis_beta(gen)*r, dis_gamma(gen)*r, n));

	}
	//cout << FilterParticles.size()<<std::endl;
}

/*
	Motion Model of the Particle filter
 */


void ParticleFilter::MotionModel(){

	Eigen::MatrixXd samples;
	Particle p;

	for(int i = 0; i < FilterParticles.size(); i++){

		samples = MultivariateGaussian(FilterParticles.at(i).GetX(),
				FilterParticles.at(i).GetY(),
				FilterParticles.at(i).GetZ(),
				FilterParticles.at(i).GetAlpha(),
				FilterParticles.at(i).GetBeta(),
				FilterParticles.at(i).GetGamma());

		for(int j = 0; j < nn; j++){

			p.SetX(samples(0,j));
			p.SetY(samples(1,j));
			p.SetZ(samples(2,j));
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

void ParticleFilter::MeasurementModel(){

	Models model;
	std::vector<cv::Point2f> left;
	std::vector<cv::Point2f> right;

	double D_Left, D_Right;
	double D_max = 279; double D_min = 0;

	double Normalizer_Left, Normalizer_Right;
	double p_Left, p_Right;
	double lambda = 0.005;

	for(int  i = 0; i < FilterParticlesWithCovariance.size(); i++){

		model.Cylinder(FilterParticlesWithCovariance.at(i),0.0175, 0.07);
		double a = GetTimeMs64();
		NewcamModel.ProjectToCameraPlane(model.Get_ModelPoints());
		double b = GetTimeMs64();

		//std::cout << b - a << std::endl;
		

		left = NewcamModel.GetProjectedModelPointsLeft();
		right = NewcamModel.GetProjectedModelPointsRight();

		for(int k = 0; k < (left.size() > right.size() ? right.size() : left.size()); k++){

			if(left.at(k).x > 0 && left.at(k).x < 280 && right.at(k).x > 0 && right.at(k).x < 280
					&& left.at(k).y > 0 && left.at(k).y < 220 && right.at(k).y > 0 && right.at(k).y < 220){


				D_Left =(double) NewFrame.GetLeftDistanceFrame().at<float>(left.at(k).x, left.at(k).y);
				D_Right = (double) NewFrame.GetRightDistanceFrame().at<float>(right.at(k).x, right.at(k).y);

				Normalizer_Left = 1/(1-exp(-lambda*D_max));
				p_Left = Normalizer_Left*lambda*exp(-lambda*D_Left);

				Normalizer_Right = 1/(1-exp(-lambda*D_max));
				p_Right = Normalizer_Right*lambda*exp(-lambda*D_Right);

				FilterParticlesWithCovariance.at(i).SetProb(0.5*p_Left + 0.5*p_Right);


				/*					std::cout << "D_Left     D_Right     NORM      p_Left      p_Right" << std::endl;
					std::cout << D_Left << "    " << D_Right  << "     " <<Normalizer_Left << "   " << p_Left  <<"   " << p_Right << std::endl;	*/

			}
		}	


		left.clear();
		right.clear();
	}
}

void ParticleFilter::Resampling(){

	//Sumatory of all the probabilities
	double sum_w = 0;

	for(int h = 0; h < nparticles*nn; h++){

		sum_w += FilterParticlesWithCovariance.at(h).GetProb();

	}

	double M = 20; //Number of samples to draw
	double r = static_cast<double> (rand())/(static_cast<double>(RAND_MAX)/(sum_w/M));	
	int i = 0;
	double U,c;

	c = FilterParticlesWithCovariance.at(0).GetProb();

	for(double m = 0; m < M; m++){

		U = r + (m)*(sum_w/M);

		while(U > c){

			i  = i + 1;

			c = c + FilterParticlesWithCovariance.at(i).GetProb(); 

		}

		/*std::cout << "Particle chosen: " << i
				<< "   with probability: " << FilterParticlesWithCovariance.at(i).GetProb()<<std::endl;*/

		FilterParticles.push_back(FilterParticlesWithCovariance.at(i));

	}
	FilterParticlesWithCovariance.clear();

	//Generate randomly the rest of particles

	float x_min = 0; 		float x_max = 0.05;
	float y_min = 0; 		float y_max = 0.05;
	float z_min = 0; 		float z_max = 0.05;
	float alpha_min = 0;	float alpha_max = 360;
	float beta_min = 0; 	float beta_max = 360;
	float gamma_min = 0; 	float gamma_max = 360;

	//Randomly generation in the limits x,y,z,a,b,c
	std::uniform_real_distribution<double> dis_x(x_min, x_max);
	std::uniform_real_distribution<double> dis_y(y_min, y_max);
	std::uniform_real_distribution<double> dis_z(z_min, z_max);
	std::uniform_real_distribution<double> dis_alpha(alpha_min, alpha_max);
	std::uniform_real_distribution<double> dis_beta(beta_min, beta_max);
	std::uniform_real_distribution<double> dis_gamma(gamma_min, gamma_max);

	for(int h = M; h < nparticles; h++){

		FilterParticles.push_back(Particle(dis_x(gen), dis_y(gen), dis_z(gen), dis_alpha(gen)*r, dis_beta(gen)*r, dis_gamma(gen)*r, h));
	}
}


/*
  Draw nn samples from a size-dimensional normal distribution
  with a specified mean and covariance
 */

Eigen::MatrixXd ParticleFilter::MultivariateGaussian(float x, float y, float z, float a, float b, float c){

	Eigen::internal::scalar_normal_dist_op<double> randN; // Gaussian functor
	Eigen::internal::scalar_normal_dist_op<double>::rng.seed(1); // Seed the rng

	// Define mean and covariance of the distribution
	Eigen::VectorXd mean(size);       
	Eigen::MatrixXd covar(size,size);

	mean  <<  x, y, z, a, b, c;
	covar <<  .05, 0, 0, 0, 0, 0,
			0, .05, 0, 0, 0, 0,
			0, 0, .05, 0, 0, 0,
			0, 0, 0, .05, 0, 0,
			0, 0, 0, 0, .05, 0,
			0, 0, 0, 0, 0, .05;

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






