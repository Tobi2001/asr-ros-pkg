//============================================================================
// Name        : Intermediate.cpp
// Author      : Jocelyn Borella
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "Domain.h"



int main(int argc, char *argv[]) {

	if(argc > 1)
	{
		std::cout << "begin Domain" << std::endl;
		Domain domain = Domain(argv[1]);
		std::list<objectTuple> intermediateObjectsList = domain.getIntermediateOjects();
		domain.publishLogs();
		std::cout << "end domain" << std::endl;
	}
	return 0;
}
