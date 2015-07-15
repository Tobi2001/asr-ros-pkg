/*
 * IObjects.cpp
 *
 *  Created on: 18 mars 2015
 *      Author: jocelyn
 */

#include "IObjects.h"


IObjects::IObjects():ObjectName("")
{
	Average = PointP(0,0,0,-1); //-1 is the ID for AveragePos point
	ObjectsCount= 0;
	PosVar = 0.0;
	AverageDistanceToOtherObjects = 0.0;
	PresenceInScene = 0.0;
	NormalizedPosVar = 0.0;
	NormalizedAverageDistanceToOtherObjects = 0.0;
	rankValue = 0.0;
	RankingMethod = 0;
	Alpha = 0;
	Beta = 0;
	Gamma = 0;
}

IObjects::IObjects(std::string objectName, int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry)
:ObjectName(objectName),RankingMethod(rankingMethodEntry),Alpha(alphaEntry),Beta(betaEntry),Gamma(gammaEntry)
{
	Average = PointP(0,0,0,-1); //-1 is the ID for AveragePos point
	ObjectsCount= 0;
	PosVar = 0.0;
	AverageDistanceToOtherObjects = 0.0;
	PresenceInScene = 0.0;
	NormalizedPosVar = 0.0;
	NormalizedAverageDistanceToOtherObjects = 0.0;
	rankValue = 0.0;
}

IObjects::~IObjects()
{

}

void IObjects::CalcAveragePos()
{
	double mx = 0;
	double my = 0;
	double mz = 0;
	//To get the average pos we "sum" the points, that compose the Object.
	for (std::vector<PointP>::iterator it = PointList.begin() ; it != PointList.end(); ++it)
		{
				mx += it->getX();
				my += it->getY();
				mz += it->getZ();
		}
	Average.updatePoint(mx/ObjectsCount,my/ObjectsCount,mz/ObjectsCount); //divide by the number of points recorded for the object.
	std::cout << " Object " << this->ObjectName << " x " << Average.getX()
			<< " y " << Average.getY() << " z " << Average.getZ() << std::endl;
}

void IObjects::CalcPosVar()
{

	double vx;
	double vy;
	double vz;
	//for each point, calc the average distance to average position.
	for (std::vector<PointP>::iterator it = PointList.begin() ; it != PointList.end(); ++it)
	{
		vx = pow(it->getX() - Average.getX(),2);
		vy = pow(it->getY() - Average.getY(),2);
		vz = pow(it->getZ() - Average.getZ(),2);
		std::cout << " object " << this->ObjectName << " vx " << vx << " vy " << vy << " vz " << vz << std::endl;
		PosVar += sqrt(vx + vy + vz);
		std::cout << " object " << this->ObjectName << " PosVar " << PosVar << std::endl;

	}
	PosVar = PosVar/ObjectsCount; //divide result by number of points.
}

void IObjects::rank()
{
	//Average the different criteria to get the rank value ( between 0 and 1 ) for the object.
	if(RankingMethod == 0)
	{
		rankValue = Alpha*PresenceInScene / (Beta*NormalizedPosVar * Gamma*NormalizedAverageDistanceToOtherObjects);
	}
	else if(RankingMethod == 1)
	{
		rankValue = Alpha*PresenceInScene + Beta/NormalizedPosVar + Beta/NormalizedAverageDistanceToOtherObjects;
	}
}

void IObjects::publishLogs(std::string filePath)
{
	std::ofstream file(filePath, std::ios::out | std::ios::app);
	if(file)
	{
		file << std::endl <<"Object " << this->ObjectName << std::endl;
		file << "Pos Var " << PosVar << std::endl;
		file << "Presence in scene " << PresenceInScene << std::endl;
		file << "AverageDistanceToOtherObjects " << AverageDistanceToOtherObjects << std::endl;
		file << "Normalized Pos " << NormalizedPosVar << std::endl;
		file << "Normalized distance " << NormalizedAverageDistanceToOtherObjects << std::endl;
		file << "Rank value " << rankValue << std::endl;
		file.close();
	}
}
