/*
 * Domain.h
 *
 *  Created on: 13 avr. 2015
 *      Author: jocelyn
 */

#ifndef DOMAIN_H_
#define DOMAIN_H_

#include "Evaluator.h"
#include"Scene.h"

class Domain {
private:
	std::string DomainName;
	std::vector<Scene> SceneList;
	const char * ConfigFilePath;
	double GainValueThreshold;
	int RankingMethod;
	double Alpha;
	double Beta;
	double Gamma;
	std::list<objectTuple> IntermediateOjects;

public:
	Domain();
	Domain(const char * configFilePath);
	virtual ~Domain();

	void calcIntermediateObjectsForDomain();
	std::vector<Scene> getDomainFromConfigFile();
	void publishLogs();
	std::list<objectTuple> getIntermediateOjects() const{return this->IntermediateOjects;};

};

#endif /* DOMAIN_H_ */
