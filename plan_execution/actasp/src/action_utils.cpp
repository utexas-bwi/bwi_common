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
    vector<AspFluent> actionsOnly;
    remove_copy_if(plan.fluents.begin(),plan.fluents.end(),back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    plans.emplace_back(vector<AspAtom>(), actionsOnly);
  }

  return plans;
}

Plan actions_to_plan(const std::list<std::unique_ptr<Action>> &plan) {
  auto actIt = plan.begin();
  set<AspFluent> fluents;

  for (int timeStep=0; actIt != plan.end(); ++actIt, ++timeStep) {
    fluents.insert((*actIt)->toFluent(timeStep));
  }

  return Plan({}, {}, std::vector<AspFluent>(fluents.begin(), fluents.end()));
}

ActionSet actionMapToSet(const std::map<std::string, ActionFactory>& actionMap) {

  ActionSet fluents;

  for (const auto &pair: actionMap) {
    // Put the real name in and a fake number of parameters
    fluents.insert(AspFluent(pair.first, {}));
  }
  return fluents;
}

ActionSet actionMapToSet(const std::map<std::string, Action *>& actionMap) {

  ActionSet fluents;
  for (const auto &pair: actionMap)
    fluents.insert(AspFluent(pair.second->toFluent(0)));

  return fluents;
}


AspFluent with_timestep(const AspFunction &fluent, uint32_t timestep) {
  const auto &params = fluent.getArguments();
  return AspFluent(fluent.getName(),{params.begin(), params.end() - 1}, fluent.getNegation(), timestep);
}

AspFluent with_timestep(const AspFunction &fluent, Variable timestep) {
  const auto &params = fluent.getArguments();
  return AspFluent(fluent.getName(),{params.begin(), params.end() - 1}, fluent.getNegation(), timestep);
}

AspRule add_timestep(const AspRule &rule, uint32_t timestep) {
  LiteralContainer head;
  LiteralContainer body;
  for (const auto &atom: rule.head_fluents()) {
    head.push_back(with_timestep(atom, timestep).literal_clone());
  }
  for (const auto &literal: rule.body_fluents()) {
    body.push_back(with_timestep(literal, timestep).literal_clone());
  }
  return {head, body};
}

AspRule add_timestep(const AspRule &rule, Variable timestep) {
  LiteralContainer head;
  LiteralContainer body;
  for (const auto &atom: rule.head_fluents()) {
    head.push_back(with_timestep(atom, timestep).literal_clone());
  }
  for (const auto &literal: rule.body_fluents()) {
    body.push_back(new AspFluent(with_timestep(literal, timestep)));
  }
  return {head, body};
}

vector<AspRule> make_goal_all_true(const std::vector<AspFluent> &fluents) {
  vector<AspRule> goal;
  for (const auto &fluent: fluents) {
    LiteralContainer body;
    auto negation = fluent.getNegation();
    negation.insert(negation.begin(),Default);
    AspFluent negated = {fluent.getName(), fluent.getArguments(), negation};
    body.push_back(new AspFluent(negated));
    TermContainer args;
    args.push_back(new Variable("n"));
    body.push_back(new AspFunction("query", args));
    goal.push_back(AspRule({}, body));
  }

  return goal;
}

}
