#include <actasp/AnswerSet.h>

#include <actasp/Action.h>

#include <algorithm>

using namespace std;

namespace actasp {

// AnswerSet::AnswerSet(bool satisfied,const std::set<actasp::AspFluent>& fluents) throw () :
// 	satisfied(satisfied),
// 	fluents(fluents.begin(),fluents.end())  {}

bool AnswerSet::isSatisfied() const throw() {
	return satisfied;
}

bool AnswerSet::contains(const actasp::AspFluent& fluent) const throw() {
  
  pair<FluentSet::const_iterator, FluentSet::const_iterator> bounds = 
          equal_range(fluents.begin(), fluents.end(), fluent, TimeStepComparator());
  
  FluentSet::const_iterator element = find(bounds.first, bounds.second, fluent); 
	return element != bounds.second;
}

static void clearPlan(std::list<actasp::Action*>& plan) {
	std::list<actasp::Action*>::iterator planIt = plan.begin();
	for(; planIt != plan.end(); ++planIt)
		delete (*planIt);
}

std::list<Action *> AnswerSet::instantiateActions(const std::map<std::string, actasp::Action*> &actionMap) const
									throw (std::logic_error) {

	list<Action *> plan;
	unsigned int maxTimeStep = 0;

	FluentSet::const_iterator fluentIt = fluents.begin();

	for (; fluentIt != fluents.end(); ++fluentIt) {
		
		map<string, Action * >::const_iterator actIt = actionMap.find(fluentIt->getName());
		
		if (actIt != actionMap.end()) {
			plan.push_back(actIt->second->cloneAndInit(*fluentIt));
			maxTimeStep = std::max(maxTimeStep,fluentIt->getTimeStep());
		} 
		//if a fluent is not a known action, just ignore it.
	}
	
	if (maxTimeStep > 0 && maxTimeStep > plan.size()) {
				clearPlan(plan);
				throw logic_error("AnswerSet: the plan is missing an action for some time step. Check the list of actions shown in the plan query.");
	}

	return plan;
}

std::set<actasp::AspFluent> AnswerSet::getFluentsAtTime(unsigned int timeStep) const throw() {
	
	//create fake fluent with the required time step
	AspFluent fake("-",vector<string>(),timeStep);
  
  pair<FluentSet::const_iterator, FluentSet::const_iterator> bounds = equal_range(fluents.begin(), fluents.end(),fake, TimeStepComparator());
	
	return set<AspFluent>(bounds.first,bounds.second);
}

unsigned int AnswerSet::maxTimeStep() const throw() {
  return fluents.rbegin()->getTimeStep();
}


}