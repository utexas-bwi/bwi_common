#include <actasp/AnswerSet.h>

#include <actasp/Action.h>

#include <algorithm>

using namespace std;

namespace actasp {

// AnswerSet::AnswerSet(bool satisfied,const std::set<actasp::AspFluent>& fluents) throw () :
// 	satisfied(satisfied),
// 	fluents(fluents.begin(),fluents.end())  {}

bool AnswerSet::isSatisfied() const noexcept {
	return satisfied;
}

bool AnswerSet::contains(const actasp::AspFluent& fluent) const noexcept {
  
  pair<FluentSet::const_iterator, FluentSet::const_iterator> bounds = 
          equal_range(fluents.begin(), fluents.end(), fluent, TimeStepComparator());
  
  FluentSet::const_iterator element = find(bounds.first, bounds.second, fluent); 
	return element != bounds.second;
}

static void clearPlan(std::list<actasp::Action::Ptr>& plan) {
	plan.clear();
}

std::list<Action::Ptr> AnswerSet::instantiateActions(const std::map<std::string, actasp::Action*> &actionMap) const
									throw (std::logic_error) {

	list<Action::Ptr> plan;
	unsigned int maxTimeStep = 0;

	FluentSet::const_iterator fluentIt = fluents.begin();

	for (; fluentIt != fluents.end(); ++fluentIt) {
		
		auto actIt = actionMap.find(fluentIt->getName());
		
		if (actIt != actionMap.end()) {
			plan.push_back(Action::Ptr(actIt->second->cloneAndInit(*fluentIt)));
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

std::set<actasp::AspFluent> AnswerSet::getFluentsAtTime(unsigned int timeStep) const noexcept {
	
	//create fake fluent with the required time step
	AspFluent fake("-",vector<string>(),timeStep);
  
  pair<FluentSet::const_iterator, FluentSet::const_iterator> bounds = equal_range(fluents.begin(), fluents.end(),fake, TimeStepComparator());
	
	return set<AspFluent>(bounds.first,bounds.second);
}

unsigned int AnswerSet::maxTimeStep() const throw(std::logic_error) {
  if(fluents.empty())
    throw logic_error("maxTimeStep() invoked on an  empty answer set, which therefore has not time step at all");
  
  return fluents.rbegin()->getTimeStep();
}


}