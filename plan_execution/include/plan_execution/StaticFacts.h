#ifndef plan_exec_StaticFacts_h__guard
#define plan_exec_StaticFacts_h__guard

#include "actasp/AspAtom.h"
#include "actasp/AspKR.h"

#include <list>

namespace plan_exec {

struct StaticFacts {
	
	static void retrieveStaticFacts(actasp::AspKR *reasoner, const std::string& domain_directory);
	static std::list<actasp::AspAtom> staticFacts();
	
private:

	static std::list<actasp::AspAtom> static_facts;

};

}

#endif

