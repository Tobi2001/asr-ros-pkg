/*
 * Scene.h
 *
 *  Created on: 31 mars 2015
 *      Author: jocelyn
 */

#ifndef SCENE_H_
#define SCENE_H_

#include <iostream>
#include <sqlite3.h>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple.hpp>
#include <math.h>
#include "IObjects.h"

class Scene {
private:
	std::map<std::string,IObjects> ObjectMap;
	std::string DbName;
	std::string SceneName;
	int SceneObjectsCount;
	int RankingMethod;
	double Alpha;
	double Beta;
	double Gamma;
public:
	Scene();
	Scene(std::string name, std::string sceneName, int rankingMethodEntry, double alphaEntry, double betaEntry, double gammaEntry);
	virtual ~Scene();

	std::string const getSceneName(){return this->SceneName;}
	std::string const getDbName(){return this->DbName;}
	std::map<std::string,IObjects> const getObjectMap(){return this->ObjectMap;}
	void getObjectFromDb();
	void getNumberSet();

	void calcAveragePositionForEachObject();
	void getObjectAverageDistance();
	void calcPresenceInSceneForEachObject();
	void DisplayStats();
	void normalize();
	void rank();
	void publishLogs(std::string filePath);
};

#endif /* SCENE_H_ */
