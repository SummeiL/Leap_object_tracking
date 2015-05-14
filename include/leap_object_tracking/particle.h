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




public:

	//Constructors and Destructor
	Particle(){ 
		x = 0; y = 0; z = 0;
		alpha = 0; beta = 0; gamma = 0;
		id = -1;
	}

	Particle(float x, float y, float z, float alpha, float beta, float gamma, float id){

		this->x = x;
		this->y = y;
		this->z = z;
		this->alpha = alpha;
		this->beta = beta;
		this->gamma = gamma;
		this->id = id;
	}

	~Particle(){}

	//Get and Set Methods
	void Setid(float id){ this->id = id;}
	void SetX(float x){ this->x = x;}
	void SetY(float y){ this->y = y;}
	void SetZ(float z){ this->z = z;}
	void SetAlpha(float alpha){ this->alpha = alpha;}
	void SetBeta(float beta){ this->beta = beta;}
	void SetGamma(float gamma){ this->gamma = gamma;}

	float Getid(){ return id;}
	float GetX(){return x;}
	float GetY(){return y;}
	float GetZ(){return z;}
	float GetAlpha(){return alpha;}
	float GetBeta(){return beta;}
	float GetGamma(){return gamma;}	
};

#endif