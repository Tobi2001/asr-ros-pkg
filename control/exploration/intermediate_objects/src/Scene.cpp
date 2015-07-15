/*
 * Scene.cpp
 *
 *  Created on: 31 mars 2015
 *      Author: jocelyn
 */

#include "Scene.h"

Scene::Scene()
{
	// TODO Auto-generated constructor stub
	SceneObjectsCount = 0;
	DbName = "void";
	SceneName = "void";
	RankingMethod = 0;
	Alpha = 0;
	Beta = 0;
	Gamma = 0;
}

Scene::Scene(std::string name, std::string sceneName, int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry)
:DbName(name),SceneName(sceneName),RankingMethod(rankingMethodEntry),Alpha(alphaEntry),Beta(betaEntry),Gamma(gammaEntry)
{
	getNumberSet();
}

Scene::~Scene()
{
	// TODO Auto-generated destructor stub
}

void Scene::getObjectFromDb()
{
	sqlite3 *database;
	std::cout << "DB NAME "<< DbName << " SCene name " << SceneName << std::endl ;
	sqlite3_open(DbName.c_str(), &database);
	sqlite3_stmt *statement;
	std::cout << "Object map size " <<ObjectMap.size() << std::endl;
	std::cout << sqlite3_prepare_v2(database, "SELECT type,px,py,pz,setId FROM recorded_objects;", -1, &statement, 0) << std::endl;
	if(sqlite3_prepare_v2(database, "SELECT type,px,py,pz,setId FROM recorded_objects;", -1, &statement, 0) == SQLITE_OK)
	{
		std::cout << "database OPEN " <<std::endl;

		int cols = sqlite3_column_count(statement);
		int result = 0;
		while(true)
		{
			result = sqlite3_step(statement);
			if(result == SQLITE_ROW)
			{
				std::string name;
				PointP pointP;
				for(int col = 0; col < cols; col++)
				{
					std::string s = (char*)sqlite3_column_text(statement, col);
					//std::cout << s << std::endl; //do something with it
					if (col == 0)
					{
						name = s;
						if(this->ObjectMap.find(s)== this->ObjectMap.end())
						{
							this->ObjectMap[s] = IObjects(s, RankingMethod, Alpha, Beta, Gamma);
							std::cout << "scene name " << SceneName <<" + " << s << std::endl;
						}
					}
					else if(col == 1)
					{
						pointP.setX(boost::lexical_cast<double>(s));
					}
					else if(col == 2)
					{
						pointP.setY(boost::lexical_cast<double>(s));
					}
					else if(col ==3)
					{
						pointP.setZ(boost::lexical_cast<double>(s));
					}
					else if(col ==3)
					{
						pointP.setSetId(boost::lexical_cast<int>(s));
					}
				}
				this->ObjectMap[name].AddPoint(pointP);
			}
			else
			{
				break;
			}
		}

		sqlite3_finalize(statement);
	}

	sqlite3_close(database);
	std::cout << "Object map size " <<ObjectMap.size() << std::endl;
}


void Scene::getNumberSet()
{
	sqlite3 *database;
	sqlite3_open(DbName.c_str(), &database);
	sqlite3_stmt *statement;
	if(sqlite3_prepare_v2(database, "SELECT COUNT(DISTINCT setId) FROM recorded_objects;", -1, &statement, 0) == SQLITE_OK)
	{
		int cols = sqlite3_column_count(statement);
		int result = sqlite3_step(statement);
		if(result == SQLITE_ROW)
		{
			for(int col = 0; col < cols; col++)
			{
				std::string s = (char*)sqlite3_column_text(statement, col);
				SceneObjectsCount = boost::lexical_cast<int>(s);
			}
		}
		sqlite3_finalize(statement);
	}
	sqlite3_close(database);
}

void Scene::calcAveragePositionForEachObject()
{
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
		it->second.CalcAveragePos();
		it->second.CalcPosVar();
	}
}

void Scene::getObjectAverageDistance()//Work in progress...
{
	bool safetyTest = false;

	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it) //Parcours liste Obj
	{
		for(int j=0; j<it->second.getObjetsCount(); j++) //Parcours point d'un objet
		{
			double distance=0;
			int counter=0;
			for (std::map<std::string,IObjects>::iterator it2=ObjectMap.begin(); it2!=ObjectMap.end(); ++it2) // Parcours list obj pr trouver autre ob
			{
				if(it2->first != it->first)  //Si les obj ne sont pas les mêmes
				{
					safetyTest = true;
					for(int k=0; k<it2->second.getObjetsCount(); k++) // Parcours des pts des autres objets
					{
						if((it2->second.getPoint(k)).getSetId() == (it->second.getPoint(j)).getSetId())
						{
							distance += it->second.getPoint(j).calcDistance(it2->second.getPoint(k));
							counter++;
						}
					}
				}
			}
			if(safetyTest==true){it->second.AddDistance(distance/counter);safetyTest=false;}
		}
		it->second.AverageDistance();
	}
}

void Scene::calcPresenceInSceneForEachObject()
{

	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
		double temp = (boost::lexical_cast<double>(it->second.getObjetsCount())/boost::lexical_cast<double>(this->SceneObjectsCount));
		std::cout << "Scene::calcPresenceInSceneForEachObject : getObjetsCount " << it->second.getObjetsCount()
				<< " this->SceneObjectsCount " << this->SceneObjectsCount
				<< "   (it->second.getObjetsCount()/this->SceneObjectsCount)*1.0   "
				<< temp << std::endl;
		it->second.setPresenceInScene(boost::lexical_cast<double>(it->second.getObjetsCount())/boost::lexical_cast<double>(this->SceneObjectsCount));
	}
}

void Scene::DisplayStats()
{
	std::cout << "Scene : " << SceneName << std::endl;
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
		it->second.DisplayStats();
	}
}


void Scene::normalize()
{
	double minValueDist=ObjectMap.begin()->second.getAverageDistanceToOtherObjects();
	double minValuePos=ObjectMap.begin()->second.getPosVar();
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
		if(it->second.getAverageDistanceToOtherObjects() < minValueDist)
		{
			minValueDist = it->second.getAverageDistanceToOtherObjects();
		}
		if(it->second.getPosVar() < minValuePos)
		{
			minValuePos = it->second.getPosVar();
		}
	}
	std::cout << "Min value pos " << minValuePos << " min value dist " << minValueDist << std::endl;
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
		double temp1 = (it->second.getAverageDistanceToOtherObjects()-minValueDist);
		std::cout << " getAverageDistanceToOtherObjects -minValueDist " <<temp1 << std::endl;

		(it->second).setNormalizedAverageDistanceToOtherObjects(temp1);
		(it->second).setNormalizedPosVar((it->second.getPosVar()-minValuePos));
	}

	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
	std::cout << " getAverageDistanceToOtherObjects " << it->second.getAverageDistanceToOtherObjects() << std::endl;
	}
	double maxValueDist=ObjectMap.begin()->second.getNormalizedAverageDistanceToOtherObjects();
	double maxValuePos=ObjectMap.begin()->second.getNormalizedPosVar();
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
		std::cout << " it->second.getNormalizedAverageDistanceToOtherObjects() " << it->second.getNormalizedAverageDistanceToOtherObjects() << std::endl;
		if(it->second.getNormalizedAverageDistanceToOtherObjects() > maxValueDist)
		{
			maxValueDist = it->second.getNormalizedAverageDistanceToOtherObjects();
			std::cout << "max value dist " << maxValueDist << std::endl;
		}
		if(it->second.getNormalizedPosVar() > maxValuePos)
		{
			maxValuePos = it->second.getNormalizedPosVar();
		}
	}
	std::cout << "Max value pos " << maxValuePos << " max value dist " << maxValueDist << std::endl;

	double valueDistCoef = 1/maxValueDist;
	double posVarCoef = 1/maxValuePos;
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
		{
			it->second.setNormalizedAverageDistanceToOtherObjects(it->second.getNormalizedAverageDistanceToOtherObjects()*valueDistCoef+0.1);
			it->second.setNormalizedPosVar(it->second.getNormalizedPosVar()*posVarCoef+0.1);
		}

}

void Scene::rank()
{
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
		{
			it->second.rank();
		}
}

void Scene::publishLogs(std::string filePath)
{
	std::ofstream file(filePath, std::ios::out | std::ios::app);
	if(file)
	{
		file << std::endl << std::endl <<"Scene " << this->SceneName << std::endl;
		file.close();
	}
	for (std::map<std::string,IObjects>::iterator it=ObjectMap.begin(); it!=ObjectMap.end(); ++it)
	{
		it->second.publishLogs(filePath);
	}
}


