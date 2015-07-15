/*
 * Evaluator.h
 *
 *  Created on: 27 juin 2015
 *      Author: jocelyn
 */

#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include<string>
#include<tuple>
#include<list>
#include<map>
#include <fstream>


typedef std::tuple<double,std::string,std::string> evalulatorTuple;
typedef std::tuple<double,std::string> objectTuple;

class Evaluator {
private:
	bool validation(std::list<objectTuple> listToValidate);
	bool eval(std::list<objectTuple> oldList, std::list<objectTuple> newList);
	double calcAverageF(std::list<objectTuple> list);
	int objectListSize;
	std::map<std::string,std::list<std::string> > sceneMap;
	std::list<objectTuple> processedObjectList;
	double GainValueThreshold;

public:
	Evaluator();
	Evaluator(std::list<evalulatorTuple> objectListEntry, std::list<std::string> sceneListEntry, double GainValueThresholdEntry);
	virtual ~Evaluator();
	std::list<objectTuple> getIntermediateObjects();

};

#endif /* EVALUATOR_H_ */
