
#include "plan_execution/msgs_utils.h"

#include <algorithm>
#include <iterator>
#include <boost/lexical_cast.hpp>

using namespace std;

namespace plan_exec {

actasp::AspFluent TranslateFluent::operator()(const plan_execution::AspFluent& bwiFluent) {
  vector<actasp::AspAtom::Argument> arguments(bwiFluent.variables.size());
  for (const auto &string_var: bwiFluent.variables) {
    try {
      // Try to interpret as int
      // Note that this means string variables must be unambigously string-y or they'll be parsed as ints!
      int asInt = std::stoi(string_var);
      arguments.emplace_back(asInt);
    } catch (std::invalid_argument &e) {
      arguments.emplace_back(string_var);
    }
  }
  return actasp::AspFluent(bwiFluent.name,arguments,bwiFluent.timeStep);
}

plan_execution::AspFluent TranslateFluent::operator()(const actasp::AspFluent& actaspFluent) {

  plan_execution::AspFluent bwiFluent;
  bwiFluent.name = actaspFluent.getName();
  bwiFluent.timeStep = actaspFluent.getTimeStep();
  bwiFluent.variables.reserve(actaspFluent.getArguments().size());
  transform(actaspFluent.getArguments().begin(), actaspFluent.getArguments().end(),
      bwiFluent.variables.begin(), [](const actasp::AspAtom::Argument &as_variant){
    return boost::lexical_cast<std::string>(as_variant);
  });

  return bwiFluent;
}


actasp::AspRule TranslateRule::operator()(const plan_execution::AspRule& bwiRule) {
  actasp::AspRule actaspRule;

  transform(bwiRule.head.begin(), bwiRule.head.end(), back_inserter(actaspRule.head), TranslateFluent());
  transform(bwiRule.body.begin(), bwiRule.body.end(), back_inserter(actaspRule.body), TranslateFluent());


  return actaspRule;
}

plan_execution::AspRule TranslateRule::operator()(const actasp::AspRule& actaspRule) {
  plan_execution::AspRule bwiRule;

  transform(actaspRule.head.begin(), actaspRule.head.end(), back_inserter(bwiRule.head), TranslateFluent());
  transform(actaspRule.body.begin(), actaspRule.body.end(), back_inserter(bwiRule.body), TranslateFluent());

  return bwiRule;

}

actasp::AnswerSet TranslateAnswerSet::operator()(const plan_execution::AnswerSet& bwiAnswerSet) {
  
  if(!bwiAnswerSet.satisfied)
    return actasp::AnswerSet();
  
  list<actasp::AspFluent> fluents;
  transform(bwiAnswerSet.fluents.begin(), bwiAnswerSet.fluents.end(), back_inserter(fluents), TranslateFluent());
  
  return actasp::AnswerSet(fluents.begin(), fluents.end());
}

plan_execution::AnswerSet TranslateAnswerSet::operator()(const actasp::AnswerSet& actaspAnswerSet) {
  plan_execution::AnswerSet bwiAnswerSet;
  
  transform(actaspAnswerSet.fluents.begin(),actaspAnswerSet.fluents.end(),back_inserter(bwiAnswerSet.fluents),TranslateFluent());
  bwiAnswerSet.satisfied = actaspAnswerSet.isSatisfied();
  
  return bwiAnswerSet;
}




}