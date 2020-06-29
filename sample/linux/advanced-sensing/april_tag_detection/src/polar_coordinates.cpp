/**************************************************************************

           Copyright(C), 2016-2026, tao.jing All rights reserved

 **************************************************************************
   File   : polar_coordinates.cpp
   Author : tao.jing
   Date   : 19-4-11
   Brief  :
**************************************************************************/
#include "polar_coordinates.h"
#include <math.h>

void polar_coordinates::polar2rec_coordinates(double yaw)
{
	dx = r_polar*sin(theta+yaw);
	dy = r_polar*cos(theta+yaw);
}

void polar_coordinates::rotate(double yaw)
{
	polar2rec_coordinates(yaw);
}

polar_coordinates::polar_coordinates(double r_polar, double theta)
{
	polar_coordinates::r_polar = r_polar;
	polar_coordinates::theta = theta;
}

double polar_coordinates::getdx()
{
	return dx;
}

double polar_coordinates::getdy()
{
	return dy;
}