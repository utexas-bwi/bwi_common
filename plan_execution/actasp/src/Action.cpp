#include <actasp/Action.h>

#include <sstream>

using namespace std;

namespace actasp {


AspFluent Action::toFluent(unsigned int timeStep) const {
  
  if(this->getParameters().size() != this->paramNumber()) {
    //the action has not been initilized yet, using a vector of placeHolders
    TermContainer arguments;
    arguments.push_back(new IntTerm(this->paramNumber()));
    return AspFluent(this->getName(),arguments, {},timeStep);
  }
  else
    return AspFluent(this->getName(),this->getParameters(), {}, timeStep);
   
}

	
}