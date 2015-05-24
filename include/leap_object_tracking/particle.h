#ifndef PARTICLE_H
#define PARTICLE_H

///////////////////////////////////////////////////////////////////
////                                                           ////
////                          INCLUDES                         ////
////                                                           ////
///////////////////////////////////////////////////////////////////

#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"

class Particle{

private:

	float id;
	float x;
	float y;
	float z;
	float alpha;
	float beta;
	float gamma;
	double prob;




public:

	//Constructors and Destructor
	explicit Particle() : x(0), y(0), z(0), alpha(0), beta(0), gamma(0), id(-1), prob(0){ }

	explicit Particle(const float& x, const float& y, const float& z, const float& alpha, const float& beta, const float& gamma, const float &id) 
	: x(this->x), y(this->y), z(this->z), alpha(this->alpha), beta(this->beta), gamma(this->gamma), id(this->id){}

	~Particle(){}

	//Get and Set Methods
	void Setid(float id){ this->id = id;}
	void SetX(float x){ this->x = x;}
	void SetY(float y){ this->y = y;}
	void SetZ(float z){ this->z = z;}
	void SetAlpha(float alpha){ this->alpha = alpha;}
	void SetBeta(float beta){ this->beta = beta;}
	void SetGamma(float gamma){ this->gamma = gamma;}
	void SetProb(double prob){ this->prob = prob;}

	float Getid(){ return id;}
	float GetX(){return x;}
	float GetY(){return y;}
	float GetZ(){return z;}
	float GetAlpha(){return alpha;}
	float GetBeta(){return beta;}
	float GetGamma(){return gamma;}	
	float GetProb(){return prob;}
};

#endif