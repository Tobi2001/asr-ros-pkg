/*
 * PointP.h
 *
 *  Created on: 18 mars 2015
 *      Author: jocelyn
 */

#ifndef POINTP_H_
#define POINTP_H_

#include <math.h>

class PointP {
private:
	double X;
	double Y;
	double Z;
	int SetId;
public:
	PointP();
	PointP(double x, double y, double z, int setId);
	virtual ~PointP();

	double getX() const {return this->X;}
	double getY() const {return this->Y;}
	double getZ() const {return this->Z;}
	int getSetId() {return this->SetId;}

	void setX(double x) {this->X=x;}
	void setY(double y) {this->Y=y;}
	void setZ(double z) {this->Z=z;}
	void setSetId(int setId) {this->SetId=setId;}

	void updatePoint(double x, double y, double z) {this->X=x; this->Y=y; this->Z=z;};
	double calcDistance(PointP p);

};

#endif /* POINTP_H_ */
