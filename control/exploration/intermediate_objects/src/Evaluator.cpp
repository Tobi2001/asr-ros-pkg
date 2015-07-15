/*
 * Evaluator.cpp
 *
 *  Created on: 27 juin 2015
 *      Author: jocelyn
 */

#include "Evaluator.h"
#include <iostream>

bool compare_evaluator_tuple(const objectTuple &first, const objectTuple &second)
{
  return (std::get<0>(first) < std::get<0>(second));
}


Evaluator::Evaluator()
{
	// TODO Auto-generated constructor stub
	GainValueThreshold = 0;
}

Evaluator::Evaluator(std::list<evalulatorTuple> objectListEntry, std::list<std::string> sceneListEntry, double GainValueThresholdEntry):GainValueThreshold(GainValueThresholdEntry)
{
	for (const std::string &it : sceneListEntry)
	{
		sceneMap[it].push_back("root-obj");
	}

	std::list<evalulatorTuple> tempObjectList;

	while(objectListEntry.size()>0)
	{
		double fValue = 0;
		std::string objectName =std::get<2>(*(objectListEntry.begin()));
		fValue += std::get<0>(*(objectListEntry.begin()));
		sceneMap[std::get<1>(*(objectListEntry.begin()))].push_back(std::get<2>(*(objectListEntry.begin())));
		objectListEntry.pop_front();
		tempObjectList = objectListEntry;
		for (const evalulatorTuple &it : objectListEntry)
		{
			if(std::get<2>(it) == objectName)
			{
				sceneMap[std::get<1>(it)].push_back(std::get<2>(it));
				fValue += std::get<0>(it);
				tempObjectList.remove(it);
			}
		}
		processedObjectList.push_back(objectTuple(fValue,objectName));
		objectListEntry = tempObjectList;
	}
	objectListSize = processedObjectList.size();
	for (const objectTuple &it : processedObjectList)
	{
		std::cout << std::get<0>(it) << " " << std::get<1>(it) << std::endl;
	}

	for (std::map<std::string,std::list<std::string>>::iterator it=sceneMap.begin(); it!=sceneMap.end(); ++it)
	{
		std::cout << it->first << std::endl;
		for (const std::string &it2 : it->second)
		{
			std::cout << it2 << std::endl;
		}
	}
}

Evaluator::~Evaluator()
{
	// TODO Auto-generated destructor stub
}


double Evaluator::calcAverageF(std::list<objectTuple> list)
{
	double ret;
	for (const objectTuple &it : list)
	{
		ret += std::get<0>(it);
	}
	return ret/list.size();
}


bool Evaluator::validation(std::list<objectTuple> listToValidate)
{
	bool test = false;
	for (std::map<std::string,std::list<std::string>>::iterator it=sceneMap.begin(); it!=sceneMap.end(); ++it)
	{
		for (const objectTuple &it2 : listToValidate)
		{
			for (const std::string &it3 : it->second)
			{
				if(it3 == std::get<1>(it2))
				{
					test = true;
					break;
				}
			}
			if(test) break;
		}
		if(!test) return false;
	}
	return true;
}

bool Evaluator::eval(std::list<objectTuple> oldList, std::list<objectTuple> newList)
{
	if ((calcAverageF(newList) - calcAverageF(oldList))/calcAverageF(newList) < GainValueThreshold)
	{
		return true;
	}
	else
	{
		return false;
	}

}

std::list<objectTuple> Evaluator::getIntermediateObjects()
{
	objectTuple tempTuple;
	std::cout<< "get IO begin " <<std::endl;
	processedObjectList.sort(compare_evaluator_tuple);
	std::cout<< "sort " <<std::endl;
	std::list<objectTuple> temp;
	std::cout<< "iteration " <<std::endl;
	for (int i=0; i<objectListSize; i++)
	{
		temp = processedObjectList;
		tempTuple = *(processedObjectList.begin());
		processedObjectList.pop_front();

		if (validation(processedObjectList) == false)
		{
			processedObjectList.push_back(tempTuple);
			continue;
		}

		if(eval(temp, processedObjectList) == false)
		{
			break;
		}
	}
	std::cout<< "end " <<std::endl;
	for (const objectTuple &it : processedObjectList)
		{
			std::cout<< std::get<0>(it) << " " << std::get<1>(it) << std::endl;
		}
	return processedObjectList;
}


