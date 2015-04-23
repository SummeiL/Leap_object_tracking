// Inclusion guard to prevent this header from being included multiple times
#ifndef THREE_DIMENSION_POINTS_H
#define THREE_DIMENSION_POINTS_H


class Point3D{

 private:
 	
	float x;
	float y;
	float z;

 public:
	
	Point3D(float x_p, float y_p, float z_p);
	Point3D();
	~Point3D(){};
	
	void SetX(float x_p);

	void SetY(float y_p);

	void SetZ(float z_p);

	float GetX();

	float GetY();

	float GetZ();
	

};
#endif
