#include <actasp/AnswerSet.h>

#include <iterator>
#include <sstream>
#include <actasp/action_utils.h>
#include <iostream>

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


std::list<std::unique_ptr<Action>> AnswerSet::instantiateActions(const map<string, actasp::ActionFactory> &actionMap,
                                                                 ResourceManager &resourceManager) const
noexcept(false) {

  list<std::unique_ptr<Action>> plan;
  unsigned int maxTimeStep = 0;


  for (const auto &fluent: fluents) {

    auto actIt = actionMap.find(fluent.getName());

    if (actIt != actionMap.end()) {
      plan.emplace_back(actIt->second(fluent, resourceManager));
      maxTimeStep = std::max(maxTimeStep,fluent.getTimeStep());
    }
    //if a fluent is not a known action, just ignore it.
  }

  if (maxTimeStep > 0 && maxTimeStep > plan.size()) {
    AnswerSet as = planToAnswerSet(plan);
    stringstream planStream;
    copy(as.getFluents().begin(), as.getFluents().end(), ostream_iterator<string>(planStream, " "));
    std::cout << planStream.str() << std::endl;
    // Wipe out instances
    plan.clear();
    throw logic_error(
        "AnswerSet: the plan is missing an action for some time step. Check the list of actions shown in the plan query.");
  }

  return plan;
}

std::list<unique_ptr<Action>> AnswerSet::instantiateActions(const map<string, actasp::Action *> &actionMap) const
									noexcept(false) {

  list<unique_ptr<Action>> plan;
	unsigned int maxTimeStep = 0;

    auto fluentIt = fluents.begin();

	for (; fluentIt != fluents.end(); ++fluentIt) {

		auto actIt = actionMap.find(fluentIt->getName());

		if (actIt != actionMap.end()) {
            plan.emplace_back(actIt->second->cloneAndInit(*fluentIt));
			maxTimeStep = std::max(maxTimeStep,fluentIt->getTimeStep());
        }
		//if a fluent is not a known action, just ignore it.
	}

    if (maxTimeStep > 0 && maxTimeStep > plan.size()) {
        AnswerSet as = planToAnswerSet(plan);
        stringstream planStream;
        copy(as.getFluents().begin(), as.getFluents().end(), ostream_iterator<string>(planStream, " "));
        std::cout << planStream.str() << std::endl;
      plan.clear();
        throw logic_error(
                "AnswerSet: the plan is missing an action for some time step. Check the list of actions shown in the plan query.");
	}

	return plan;
}

std::set<actasp::AspFluent> AnswerSet::getFluentsAtTime(unsigned int timeStep) const noexcept {

    //create fake fluent with the required time step
	AspFluent fake("-",vector<string>(),timeStep);

    pair<FluentSet::const_iterator, FluentSet::const_iterator> bounds = equal_range(fluents.begin(), fluents.end(),fake, TimeStepComparator());

    return set<AspFluent>(bounds.first,bounds.second);
}

unsigned int AnswerSet::maxTimeStep() const noexcept(false) {
  if(fluents.empty())
    throw logic_error("maxTimeStep() invoked on an  empty answer set, which therefore has not time step at all");

    return fluents.rbegin()->getTimeStep();
}


}
