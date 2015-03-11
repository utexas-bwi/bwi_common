#include <actasp/Action.h>

#include <sstream>

using namespace std;

namespace actasp {
	
	
std::string Action::toASP(unsigned int timeStep) const {
	
	stringstream nameS;
	
	nameS << this->getName() << "(";

	for(int i=0, size=this->getParameters().size(); i<size ; ++i)
		nameS << this->getParameters()[i] << ",";

	nameS << timeStep <<")";
	
	return nameS.str();
	
}

AspFluent Action::toFluent(unsigned int timeStep) const {
  
  if(this->getParameters().size() != this->paramNumber()) {
    //the action has not been initilized yet, using a vector of placeHolders
    return AspFluent(this->getName(),vector<string>(this->paramNumber()), timeStep);
  }
  else
    return AspFluent(this->getName(),this->getParameters(), timeStep);
   
}

	
}