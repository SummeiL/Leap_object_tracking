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

	explicit Particle(const float& X, const float& Y, const float& Z, const float& a, const float& b, const float& g, const float &n) 
	: x(X), y(Y), z(Z), alpha(a), beta(b), gamma(g), id(n){}

	~Particle(){}

	//Get and Set Methods
	void Setid(float id){ this->id = id;}
	void SetX(float x){ this->x = x;}
	void SetY(float y){ this->y = y;}
	void SetZ(float z){ this->z = z;}
	void SetAlpha(float alpha){ this->alpha = alpha;}
	void SetBeta(float beta){ this->beta = beta;}
	void SetGamma(float gamma){ this->gamma = gamma;}
	void SetProb(long double prob){ this->prob = prob;}

	float Getid(){ return id;}
	float GetX(){return x;}
	float GetY(){return y;}
	float GetZ(){return z;}
	float GetAlpha(){return alpha;}
	float GetBeta(){return beta;}
	float GetGamma(){return gamma;}	
	long double GetProb(){return prob;}
};

#endif
