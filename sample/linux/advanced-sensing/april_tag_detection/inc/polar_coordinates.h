/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : polar_coordinates.h
   Author : tao.jing
   Date   : 19-4-11
   Brief  : 
**************************************************************************/

#ifndef LAND_PROJECT_POLAR_COORDINATES_H
#define LAND_PROJECT_POLAR_COORDINATES_H

class polar_coordinates
{
public:
	double r_polar;
	double theta;
	void rotate(double yaw);
	polar_coordinates(double  r_polar, double theta);
	double getdx();
	double getdy();
private:
	double dx;
	double dy;
	void polar2rec_coordinates(double yaw);
};

#endif //LAND_PROJECT_POLAR_COORDINATES_H
