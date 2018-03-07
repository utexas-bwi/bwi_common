#include <actasp/action_utils.h>


#include <algorithm>
#include <iterator>

using namespace std;

namespace actasp {

struct ActionName {
  string operator()(const AspFluent& action) const {
    return action.getName();
  }

};

IsAnAction::IsAnAction(const ActionSet& actions) {
  transform(actions.begin(), actions.end(), inserter(actionNames, actionNames.begin()), ActionName());
}

bool IsAnAction::operator()(const AspFluent& fluent) const {

  return actionNames.find(fluent.getName()) != actionNames.end();
}

AnswerSet planToAnswerSet(const std::list<Action*>& plan) {

  list<Action*>::const_iterator actIt = plan.begin();
  set<AspFluent> fluents;

  for (int timeStep=0; actIt != plan.end(); ++actIt, ++timeStep) {
    fluents.insert((*actIt)->toFluent(timeStep));
  }

  return AnswerSet(fluents.begin(), fluents.end());
}

ActionSet actionMapToSet(const std::map<std::string, Action *>& actionMap) {

  ActionSet fluents;

  std::map<std::string, Action *>::const_iterator mapIt = actionMap.begin();
  for (; mapIt != actionMap.end(); ++mapIt)
    fluents.insert(AspFluent(mapIt->second->toFluent(0)));

  return fluents;
}

}
