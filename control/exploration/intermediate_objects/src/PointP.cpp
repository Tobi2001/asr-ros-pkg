/*
 * PointP.cpp
 *
 *  Created on: 18 mars 2015
 *      Author: jocelyn
 */

#include "PointP.h"

PointP::PointP():X(0),Y(0),Z(0),SetId(0)
{

}

PointP::PointP(double x, double y, double z, int setId):X(x),Y(y),Z(z),SetId(setId)
{

}

PointP::~PointP()
{

}

double PointP::calcDistance(PointP p)
{
	return sqrt(pow((this->X)-p.getX(),2)+pow((this->Y)-p.getY(),2)+pow((this->Z)-p.getZ(),2));
}

