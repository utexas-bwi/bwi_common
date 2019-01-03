#include <actasp/action_utils.h>


#include <iterator>

using namespace std;

namespace actasp {


bool IsAnAction::operator()(const AspFluent& fluent) const {
  return actionNames.find(fluent.getName()) != actionNames.end();
}

std::list<AnswerSet> filterPlans(const std::list<AnswerSet> &unfiltered_plans, const std::set<std::string>& allActions) {

  list<AnswerSet> plans;

  for (const auto &plan: unfiltered_plans) {
    list<AspFluent> actionsOnly;
    remove_copy_if(plan.getFluents().begin(),plan.getFluents().end(),back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    plans.emplace_back(actionsOnly.begin(), actionsOnly.end());
  }

  return plans;
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
    fluents.insert(AspFluent(pair.first, std::vector<AspAtom::Argument>{}));
  }
  return fluents;
}

ActionSet actionMapToSet(const std::map<std::string, Action *>& actionMap) {

  ActionSet fluents;
  for (const auto &pair: actionMap)
    fluents.insert(AspFluent(pair.second->toFluent(0)));

  return fluents;
}

AspFluent with_timestep(const AspFluent &fluent, uint32_t timestep) {
  const auto &params = fluent.getArguments();
  return AspFluent(fluent.getName(),{params.begin(), params.end() - 1}, timestep, fluent.getNegation());
}

AspFluent with_timestep(const AspFluent &fluent, Variable timestep) {
  const auto &params = fluent.getArguments();
  return AspFluent(fluent.getName(),{params.begin(), params.end() - 1}, timestep, fluent.getNegation());
}

AspFluentRule add_timestep(const AspFluentRule &rule, uint32_t timestep) {
  std::vector<AspFluent> head;
  std::vector<AspFluent> body;
  for (const auto atom: rule.head) {
    head.push_back(with_timestep(atom, timestep));
  }
  for (const auto atom: rule.body) {
    body.push_back(with_timestep(atom, timestep));
  }
  return {head, body};
}

AspFluentRule add_timestep(const AspFluentRule &rule, Variable timestep) {
  std::vector<AspFluent> head;
  std::vector<AspFluent> body;
  for (const auto &atom: rule.head) {
    head.push_back(with_timestep(atom, timestep));
  }
  for (const auto &atom: rule.body) {
    body.push_back(with_timestep(atom, timestep));
  }
  return {head, body};
}

vector<AspFluentRule> make_goal_all_true(const std::vector<AspFluent> &fluents) {
  vector<AspFluentRule> goal;
  for (const auto &fluent: fluents) {
    AspFluentRule rule;
    auto negation = fluent.getNegation();
    negation.insert(negation.begin(),Default);
    AspFluent negated = {fluent.getName(), fluent.getArguments(), negation};
    rule.body.push_back(negated);
    rule.body.emplace_back("query", vector<AspAtom::Argument>{Variable("n")});
    goal.push_back(rule);
  }

  return goal;
}

}
