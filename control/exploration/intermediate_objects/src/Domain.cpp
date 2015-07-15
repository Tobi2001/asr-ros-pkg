/*
 * Domain.cpp
 *
 *  Created on: 13 avr. 2015
 *      Author: jocelyn
 */

#include "Domain.h"
#include "rapidxml/rapidxml.hpp"
#include <fstream>
#include <chrono>
#include <ctime>

Domain::Domain()
{
	DomainName = "";
	ConfigFilePath = "";
	GainValueThreshold = 0;
	RankingMethod = 0;
	Alpha = 0;
	Beta = 0;
	Gamma = 0;
}

Domain::Domain(const char * configFilePath):ConfigFilePath(configFilePath)
{
	SceneList = getDomainFromConfigFile();
	calcIntermediateObjectsForDomain();
}

Domain::~Domain()
{

}

void Domain::calcIntermediateObjectsForDomain()
{
	std::list<objectTuple> result;
	for (std::vector<Scene>::iterator it = SceneList.begin(); it != SceneList.end(); ++it)
	{
		std::cout << "Scene Name " << it->getSceneName() << " DB NAME " << it->getDbName() << std::endl;
		it->getObjectFromDb();
		it->calcAveragePositionForEachObject();
		it->calcPresenceInSceneForEachObject();
		it->getObjectAverageDistance();
		it->normalize();
		it->rank();
		it->DisplayStats();
	}
	std::cout<< "Evaluator begin creation " <<std::endl;
	std::list<evalulatorTuple> list;
	std::list<std::string> sceneList;
	std::cout<< "iteration begin " <<std::endl;
	for (std::vector<Scene>::iterator it = SceneList.begin(); it != SceneList.end(); ++it)
	{
		std::string sceneName = it->getSceneName();
		std::cout<< "scene name " << sceneName <<std::endl;
		sceneList.push_back(sceneName);
		std::map<std::string,IObjects> tempMap = it->getObjectMap();
		std::cout<< "2nd iteration" <<std::endl;
		for (std::map<std::string,IObjects>::iterator it=tempMap.begin(); it!=tempMap.end(); ++it)
		{
			list.push_back(evalulatorTuple(it->second.getRankValue(),sceneName,it->first));
		}
	}
	std::cout<< "Evaluator constrc " <<std::endl;
	Evaluator evaluator = Evaluator(list,sceneList,GainValueThreshold);
	IntermediateOjects = evaluator.getIntermediateObjects();
}

std::vector<Scene> Domain::getDomainFromConfigFile()
{
	std::vector<Scene> SceneListTemp;
	//We get Domain name + Scene names + DB Names from config file and store them in the scene list.
	rapidxml::xml_document<> doc;
	rapidxml::xml_node<> * root_node;
	// Read the xml file into a vector
	std::ifstream ConfigFile(this->ConfigFilePath,std::ifstream::in);
	std::vector<char> buffer((std::istreambuf_iterator<char>(ConfigFile)), std::istreambuf_iterator<char>());
	//buffer.push_back('\0');
	// Parse the buffer using the xml file parsing library into doc
	doc.parse<0>(&buffer[0]);

	// Find our root node
	root_node = doc.first_node("Domain");
	this->DomainName = root_node->first_attribute("name")->value();

	for (rapidxml::xml_node<> * param_node = root_node->first_node("Param"); param_node; param_node = param_node->next_sibling())
	{
		std::string str(param_node->first_attribute("name")->value());
		if(str == "GainValueThreshold")
		{
			GainValueThreshold = boost::lexical_cast<double>(param_node->value());
		}
		else if(str == "RankingMethod")
		{
			std::string temp(param_node->value());
			if(temp == "Mult")
			{
				RankingMethod = 0;
			}
			else if(temp == "Additive")
			{
				RankingMethod = 1;
			}
			else
			{
				RankingMethod = 1;
			}
		}
		else if(str == "Alpha")
		{
			Alpha = boost::lexical_cast<double>(param_node->value());
		}
		else if(str == "Beta")
		{
			Beta = boost::lexical_cast<double>(param_node->value());
		}
		else if(str == "Gamma")
		{
			Gamma = boost::lexical_cast<double>(param_node->value());
		}
		else
		{
			std::cout << "You are trying to use an incorrect paramater." << std::endl;
		}
	}

	for (rapidxml::xml_node<> * scene_node = root_node->first_node("Scene"); scene_node; scene_node = scene_node->next_sibling())
	{
		std::string dbName = scene_node->first_attribute("path")->value();
		std::string name = scene_node->first_attribute("name")->value();
		Scene temp = Scene(dbName, name, RankingMethod, Alpha, Beta, Gamma);
		SceneListTemp.push_back(temp);
	}

	return SceneListTemp;
}

void Domain::publishLogs()
{
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();
	std::time_t start_time = std::chrono::system_clock::to_time_t(start);
	std::string filePath = this->DomainName+".log";
	std::ofstream file(filePath, std::ios::out | std::ios::app);
	if(file)
	{
		file << std::endl <<this->DomainName << " Log file" << std::endl;
		file << "Timestamp for the Log " << std::ctime(&start_time) << std::endl;
		file << std::endl << "Intermediate Objects for The Domain " << std::endl;
		for (const objectTuple &it : IntermediateOjects)
		{
			file << std::get<1>(it) << " : " << std::get<0>(it) << std::endl;
		}
		file.close();
	}
	for (std::vector<Scene>::iterator it = SceneList.begin(); it != SceneList.end(); ++it)
	{
		it->publishLogs(filePath);
	}
}
