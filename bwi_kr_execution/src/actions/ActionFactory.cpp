#include "ActionFactory.h"

#include "actasp/Action.h"
#include <boost/config/no_tr1/complex.hpp>

using namespace std;
using namespace actasp;

namespace bwi_krexec {

bool ActionFactory::simulation(false);

ActionFactory::ActionFactory(Action* act) {

	//TODO warn if action already there
	bothActions().insert( make_pair(act->getName(),act));

}

ActionFactory::ActionFactory(actasp::Action *act, bool simulation) {
	
	if(simulation)
		simulatedActions().insert( make_pair(act->getName(),act));
	else
		realActions().insert( make_pair(act->getName(),act));
}


Action* ActionFactory::byName(const std::string& name) throw (std::runtime_error) {
	
	map<string, Action*>::const_iterator actIt = bothActions().find(name);
	
	if(actIt == actions().end()) {
		
		ActionMap &inUse = (simulation)? simulatedActions() : realActions();
			
		actIt = inUse.find(name);
		if(actIt == inUse.end())
			throw runtime_error("No action with name " + name);
	}

	return actIt->second->clone();
}

ActionFactory::ActionMap ActionFactory::actions() {
	ActionFactory::ActionMap actions; 
	if(simulation)
		actions.insert(simulatedActions().begin(),simulatedActions().end());
	else
		actions.insert(realActions().begin(),realActions().end());
  
  actions.insert(bothActions().begin(),bothActions().end());
	
	return actions;
}

ActionFactory::ActionMap &ActionFactory::realActions() {
	static ActionFactory::ActionMap realAct;
	return realAct;
}

ActionFactory::ActionMap &ActionFactory::simulatedActions() {
	static ActionFactory::ActionMap simulAct;
	return simulAct;
}

ActionFactory::ActionMap &ActionFactory::bothActions() {
	static ActionFactory::ActionMap bothAct;
	return bothAct;
}

void ActionFactory::setSimulation(bool value) {
	simulation = value;
}

	
}