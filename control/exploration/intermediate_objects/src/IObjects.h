/*
 * IObjects.h
 *
 *  Created on: 18 mars 2015
 *      Author: jocelyn
 */

#ifndef IOBJECTS_H_
#define IOBJECTS_H_

#include "PointP.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <math.h>


class IObjects {
private:
	std::string ObjectName;
	PointP Average;
	int ObjectsCount;
	double PosVar;
	double AverageDistanceToOtherObjects;
	double PresenceInScene;
	double NormalizedPosVar;
	double NormalizedAverageDistanceToOtherObjects;
	std::vector<PointP> PointList;
	double rankValue;
	int RankingMethod;
	double Alpha;
	double Beta;
	double Gamma;

public:
	IObjects(); //default constructor
	IObjects(std::string objectName, int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry);
	virtual ~IObjects();

	//getter
	PointP getAverage() const {return this->Average;}
	double getPosVar() const {return this->PosVar;}
	PointP getPoint(int i) const {return this->PointList[i];}
	int getObjetsCount() const {return this->ObjectsCount;}
	std::string getObjectName() const {return this->ObjectName;}
	double getAverageDistanceToOtherObjects() const {return this->AverageDistanceToOtherObjects;}
	double getRankValue() const {return this->rankValue;}
	double getNormalizedPosVar() {return this->NormalizedPosVar;}
	double getNormalizedAverageDistanceToOtherObjects() {return this->NormalizedAverageDistanceToOtherObjects;}

	//setter
	void setPresenceInScene(double presence){this->PresenceInScene = presence;}
	void setNormalizedPosVar(double value){this->NormalizedPosVar = value;}
	void setNormalizedAverageDistanceToOtherObjects(double value){ this->NormalizedAverageDistanceToOtherObjects = value;}

	void CalcAveragePos();
	void CalcPosVar();
	void AddPoint(PointP p) {this->PointList.push_back(p); ObjectsCount++;}
	void AddDistance(double d){ this->AverageDistanceToOtherObjects += d;}
	void AverageDistance(){ this->AverageDistanceToOtherObjects = this->AverageDistanceToOtherObjects/ObjectsCount;}
	void DisplayStats(){ std::cout << "Obj Name " <<ObjectName << " Pos Var " << PosVar << " Presence in scene "
		<< PresenceInScene << " AverageDistanceToOtherObjects " << AverageDistanceToOtherObjects
		<< " Normalized Pos " << NormalizedPosVar << " Normalized distance " << NormalizedAverageDistanceToOtherObjects
		<< " rank value " << rankValue <<std::endl;}
	void rank();
	void publishLogs(std::string filePath);
};

#endif /* IOBJECTS_H_ */
