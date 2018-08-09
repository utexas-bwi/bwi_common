#include <actasp/action_utils.h>


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

AnswerSet planToAnswerSet(const std::list<std::unique_ptr<Action>> &plan) {
  auto actIt = plan.begin();
  set<AspFluent> fluents;

  for (int timeStep=0; actIt != plan.end(); ++actIt, ++timeStep) {
    fluents.insert((*actIt)->toFluent(timeStep));
  }

  return AnswerSet(fluents.begin(), fluents.end());
}

ActionSet actionMapToSet(const std::map<std::string, ActionFactory>& actionMap) {

  ActionSet fluents;

  for (const auto &pair: actionMap) {
    // Put the real name in and a fake number of parameters
    fluents.insert(AspFluent(pair.first, {2,""}));
  }
  return fluents;
}

ActionSet actionMapToSet(const std::map<std::string, Action *>& actionMap) {

  ActionSet fluents;
  for (const auto &pair: actionMap)
    fluents.insert(AspFluent(pair.second->toFluent(0)));

  return fluents;
}

}
