#include <actasp/reasoners/Clingo5_2.h>

#include <actasp/asp/AspRule.h>
#include <actasp/AnswerSet.h>
#include <actasp/asp/AspAtom.h>
#include <actasp/asp/AspProgram.h>
#include <actasp/asp/AspMeta.h>
#include <actasp/asp/AspMinimize.h>
#include <actasp/action_utils.h>

#include <algorithm>
#include <iterator>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <limits>

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <actasp/filesystem_utils.h>
#include <clingo.hh>


using namespace std;
using boost::filesystem::path;

namespace actasp {



Clingo5_2::Clingo5_2(
    const std::vector<std::string> &linkFiles,
    unsigned int max_time
) noexcept :
    max_time(max_time),
    linkFiles(linkFiles),
    actions_only(true),
    incrementalVar("n") {

  Clingo::Control control;
  for (const auto &linkFile: linkFiles) {
    assert(boost::filesystem::is_regular_file(linkFile));
    control.load(linkFile.c_str());
  }
  control.ground({{"base", Clingo::SymbolSpan{}}});
  for (const auto &atom: control.symbolic_atoms()) {
    const auto symbol = atom.symbol();
    // Domains can annotate which predicates denote actions using the "action" predicate with the name of the action
    // as the sole string argument
    if (symbol.type() == Clingo::SymbolType::Function && symbol.name() == std::string("action")) {
      allActions.insert(symbol.arguments()[0].string());
    }
  }
  if (control.has_const("incvar")) {
    incrementalVar = control.get_const("incvar").string();
  }

}

std::string rule_to_string(const actasp::AspRule *rule) {
  std::stringstream sstream;
  bool first = true;
  for (const auto &headAtom: rule->head) {
    if (!first) {
      sstream << ", ";
    }
    sstream << headAtom.to_string();
    first = false;
  }
  if (!rule->body.empty()) {
    sstream << " :- ";
  }
  first = true;
  for (const auto &bodyAtom: rule->body) {
    if (!first) {
      sstream << ", ";
    }
    sstream << bodyAtom.to_string();
    first = false;
  }
  sstream << ".";
  return sstream.str();
}

// TODO: Support minimization statements
std::string minimize_to_string(const actasp::AspMinimize *minimize) {
  assert(false);
}

// TODO: Support meta statements
std::string meta_to_string(const actasp::AspMeta *minimize) {
  assert(false);
}

std::string element_to_string(const actasp::AspElement *element) {
  if (const auto r = dynamic_cast<const AspRule*>(element)) {
    return rule_to_string(r);
  } else if (const auto c = dynamic_cast<const AspMinimize*>(element)) {
    return minimize_to_string(c);
  } else if (const auto m = dynamic_cast<const AspMeta*>(element)) {
    return meta_to_string(m);
  } else {
  assert(false);
  }
}

std::string program_to_string(const actasp::AspProgram &program) {
  std::stringstream stream;
  stream << "#program " << program.getName();
  if (!program.getVariables().empty()) {
    stream << "(";
    bool past_first = false;
    for (const auto &variable: program.getVariables()) {
      if (past_first) {
        stream << ", ";
      }
      stream << variable;
      past_first = true;
    }
    stream << ")";
  }
  stream << "." << std::endl;
  for (const auto &element: program.get_elements()) {
    stream << element_to_string(element);
  }
  return stream.str();
}


struct RuleToString5_2 {

  std::string operator()(const AspRule &rule) const {

    stringstream ruleStream;

    //iterate over head
    for (int i = 0, size = rule.head.size(); i < size; ++i) {

      ruleStream << rule.head[i].to_string();

      if (i < (size - 1))
        ruleStream << " | ";
    }

    if (!(rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i = 0, size = rule.body.size(); i < size; ++i) {

      ruleStream << rule.body[i].to_string();

      if (i < (size - 1))
        ruleStream << ", ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << "." << std::endl;

    return ruleStream.str();
  }

};

struct RuleToCumulativeString5_2 {

  RuleToCumulativeString5_2(Variable timeStepVar) : timeStep(std::move(timeStepVar)) {}

  std::string operator()(const AspRule &rule) const {

    stringstream ruleStream;
    unsigned int headTimeStep = 0;

    //iterate over head
    for (int i = 0, size = rule.head.size(); i < size; ++i) {
      ruleStream << with_timestep(rule.head[i], timeStep);
      auto as_fluent = AspFluent(rule.head[i]);
      headTimeStep = std::max(headTimeStep, as_fluent.getTimeStep());

      if (i < (size - 1))
        ruleStream << " | ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i = 0, size = rule.body.size(); i < size; ++i) {
      ruleStream << rule.body[i].to_string();

      if (i < (size - 1))
        ruleStream << ", ";
    }

    if (!rule.head.empty()) {
      if (!rule.body.empty())
        ruleStream << ", ";

      ruleStream << timeStep << "=" << headTimeStep;
    }


    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << "." << endl;

    return ruleStream.str();
  }

  Variable timeStep;
};


static AnswerSet model_to_answer_set(const Clingo::Model &model) {
  cout << "Model" << endl;
  vector<AspFluent> set;
  // FIXME: This will swallow atoms that aren't fluents. We should move away from using fluents as the default representation
  for (auto &atom : model.symbols(Clingo::ShowType::All)) {
    try {
      if (atom.type() == Clingo::SymbolType::Function) {
        vector<AspAtom::Argument> variables(atom.arguments().size());
        for (const auto &argument: atom.arguments()) {
          if (argument.type() == Clingo::SymbolType::Number) {
            variables.emplace_back(argument.number());
          } else if (argument.type() == Clingo::SymbolType::String) {
            variables.emplace_back(argument.string());
          }
        }
        set.emplace_back(atom.name(), variables);
      }
      //cout << atom << endl;
    } catch (std::invalid_argument e) {
      std::cerr << "Could not parse atom: " << atom << std::endl;
    }
  }

  for (auto &atom: model.symbols(Clingo::ShowType::All)) {
    cout << atom << endl;
  }
  cout << endl << endl;
  return AnswerSet(set.begin(), set.end());
}

static string cumulativeString(const std::vector<actasp::AspRule> &query, const string &timeStepVar) {

  stringstream aspStream;
  transform(query.begin(), query.end(), ostream_iterator<std::string>(aspStream),
            RuleToCumulativeString5_2(timeStepVar));
  return aspStream.str();
}

static string aspString(const std::vector<actasp::AspRule> &query, unsigned int timeStep) {
  stringstream aspStream;
  std::vector<actasp::AspRule> rules;
  for (const auto &rule: query) {
    rules.push_back(add_timestep(rule, timeStep));
  }
  transform(query.begin(), query.end(), ostream_iterator<std::string>(aspStream), RuleToString5_2());
  return aspStream.str();
}

static inline void add(Clingo::Control &control, const actasp::AspProgram &program) {
  std::vector<const char *> as_strings;
  for (const auto &variable: program.getVariables()) {
    as_strings.push_back(variable.name.c_str());
  }
  std::stringstream stream;
  stream << "#external query(n)." << std::endl;
  for (const auto &rule: program.get_elements()) {
    stream << element_to_string(rule);
  }
  const auto program_str = stream.str();
  control.add(program.getName().c_str(), as_strings, program_str.c_str());
}


std::list<actasp::AnswerSet> Clingo5_2::minimalPlanQuery(const std::vector<actasp::AspRule> &goalRules,
                                                         unsigned int max_plan_length,
                                                         unsigned int answerset_number,
                                                         bool actions_only) const noexcept {


  list<AnswerSet> answers = makeQuery(goalRules, 0, max_plan_length, "planQuery", answerset_number);

  if (actions_only)
    return filterPlans(answers, allActions);
  else
    return answers;

}


std::list<actasp::AnswerSet> Clingo5_2::lengthRangePlanQuery(const std::vector<actasp::AspRule> &goalRules,
                                                             unsigned int min_plan_length,
                                                             unsigned int max_plan_length,
                                                             unsigned int answerset_number,
                                                             bool actions_only) const noexcept {

  std::list<actasp::AnswerSet> allplans = makeQuery(goalRules, max_plan_length, max_plan_length, "planQuery",
                                                    answerset_number);

  //clingo 3 generates all plans up to a maximum length anyway, we can't avoid the plans shorter than min_plan_length to be generated
  //we can only filter them out afterwards

  allplans.remove_if(MaxTimeStepLessThan(min_plan_length));

  if (actions_only)
    return filterPlans(allplans, allActions);
  else
    return allplans;

}

actasp::AnswerSet Clingo5_2::optimalPlanQuery(const std::vector<actasp::AspRule> &goalRules,
                                              unsigned int max_plan_length,
                                              unsigned int answerset_number, bool actions_only) const noexcept {

  list<AnswerSet> allAnswers = makeQuery(goalRules, 0, max_plan_length, "planQuery", answerset_number, true);

  if (allAnswers.empty()) {
    return {};
  }
  const AnswerSet &optimalPlan = allAnswers.front();
  if (actions_only) {
    list<AnswerSet> sets;
    sets.push_back(optimalPlan);
    return *(filterPlans(sets, allActions).begin());
  } else
    return optimalPlan;
}

AnswerSet Clingo5_2::currentStateQuery(const std::vector<actasp::AspRule> &query) const noexcept {
  // TODO: This used to be aspString(query, 0). Fix it
  list<AnswerSet> sets = makeQuery(query, 0, 0, "stateQuery", 1);

  return (sets.empty()) ? AnswerSet() : *(sets.begin());
}

struct HasTimeStepZeroInHead5_2 : unary_function<const AspRule &, bool> {

  bool operator()(const AspRule &rule) const {
    if (rule.head.empty())
      return false;
    auto as_fluent = AspFluent(rule.head[0]);
    return as_fluent.getTimeStep() == 0; //I am assuming the heads have a single fluent
    //If the that's not the case, the option --shift has to be added to clingo's command line
  }
};

std::list<actasp::AnswerSet> Clingo5_2::genericQuery(const std::vector<actasp::AspRule> &query,
                                                     unsigned int timeStep,
                                                     const std::string &fileName,
                                                     unsigned int answerSetsNumber, bool useCopyFiles) const noexcept {

  std::vector<actasp::AspRule> base;
  remove_copy_if(query.begin(), query.end(), back_inserter(base), not1(HasTimeStepZeroInHead5_2()));


  return makeQuery(query, timeStep, timeStep, fileName, answerSetsNumber);

}

std::string Clingo5_2::generateMonitorQuery(const std::vector<actasp::AspRule> &goalRules,
                                            const AnswerSet &plan) const noexcept {

  stringstream monitorQuery("", ios_base::app | ios_base::out);

  monitorQuery << "#program step(" << incrementalVar << ")." << endl;


  const AnswerSet::FluentSet &actionSet = plan.getFluents();
  auto actionIt = actionSet.begin();
  vector<AspRule> plan_in_rules;

  for (int i = 1; actionIt != actionSet.end(); ++actionIt, ++i) {
    AspFluent action(*actionIt);
    AspRule actionRule;
    actionRule.head.push_back(with_timestep(action, i));
    plan_in_rules.push_back(actionRule);
  }

  monitorQuery << cumulativeString(plan_in_rules, "n");

  return monitorQuery.str();
}

std::list<actasp::AnswerSet> Clingo5_2::monitorQuery(const std::vector<actasp::AspRule> &goalRules,
                                                     const AnswerSet &plan) const noexcept {

  //   clock_t kr1_begin = clock();

  string monitorQuery = generateMonitorQuery(goalRules, plan);

  list<actasp::AnswerSet> result = makeQuery(goalRules, plan.getFluents().size(), plan.getFluents().size(),
                                             "monitorQuery", 1);

  result.remove_if(MaxTimeStepLessThan(plan.getFluents().size()));

//   clock_t kr1_end = clock();
//   cout << "Verifying plan time: " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  return result;
}

list<AnswerSet>
Clingo5_2::makeQuery(const std::vector<AspRule> &goal, unsigned int initialTimeStep, unsigned int finalTimeStep,
                     const std::string &fileName, unsigned int answerSetsNumber, bool useCopyFiles) const noexcept {

  using S = std::string;
  using Clingo::Number;
  using Clingo::Function;
  using Clingo::TruthValue;
  using Clingo::Control;
  using ModelVec = std::vector<Clingo::SymbolVector>;
  using MessageVec = std::vector<std::pair<Clingo::WarningCode, std::string>>;
  auto logger = [](Clingo::WarningCode, char const *message) {
    cerr << message << endl;
  };

  Clingo::Control control({}, logger, 20);

  control.configuration()["solve"]["models"] = std::to_string(answerSetsNumber).c_str();

  if (useCopyFiles) {
    for (const auto &path: copyFiles) {
      control.load(path.c_str());
    }
  }
  for (const auto &path: linkFiles) {
    control.load(path.c_str());
  }

  path
  queryDir = getQueryDirectory(linkFiles, copyFiles);
  auto queryDirFiles = populateDirectory(queryDir, linkFiles, copyFiles);

  const path queryPath = (queryDir / fileName).string() + ".asp";
  std::vector<AspElement *> goal_elements;
  // FIXME: This is gross, and only safe because goal_elements will be
  // used while goal is in scope
  for (int i = 0; i < goal.size(); i++) {
    goal_elements.push_back((AspElement*)(&goal[i]));
  }
  AspProgram check("check", goal_elements, {Variable("n")});
  //AspProgram check("check", {{{},{"not query(4)"_f}}}, {Variable("n")});

  add(control, check);
  std::ofstream query_file(queryPath.string());
  query_file << program_to_string(check);
  query_file << std::endl << "#external query(n)." << std::endl;
  query_file.close();
  //const path outputFilePath = (queryDir / fileName).string() + "_output.txt";
  list<AnswerSet> allAnswers;
  int step = 0;
  Clingo::SolveResult result;
  Clingo::SolveHandle handle;
  while (step < finalTimeStep && (step == 0 || step < initialTimeStep || !result.is_satisfiable())) {
    try {
      vector<Clingo::Part> parts;
      parts.emplace_back("check", Clingo::SymbolSpan{Number(step)});
      if (step == 0) {
        parts.emplace_back("base", Clingo::SymbolSpan{});
      } else {
        parts.push_back({"step", {Number(step)}});
        control.cleanup();
      }
      control.ground(parts);
      if (step == 0) {
        control.assign_external(Function("query", {Number(step)}), TruthValue::True);
      } else {
        control.release_external(Function("query", {Number(step - 1)}));
        control.assign_external(Function("query", {Number(step)}), TruthValue::True);
      }
      for (auto atom: control.theory_atoms()) {
        std::cout << atom << std::endl;
      }
      for (auto atom: control.symbolic_atoms()) {
        std::cout << atom.symbol() << std::endl;
      }
      std::cout << "_------ END PROGRAM" << std::endl;
      handle = control.solve();
      result = handle.get();
      step++;
      std::cout << handle.get() << std::endl;
    }
    catch (exception const &e) {
      cerr << "Grounding failed with: " << e.what() << endl;
      return {};
    }
  }
  for (const auto &model : handle) {
    allAnswers.push_back(model_to_answer_set(model));
  }

  return allAnswers;
}


std::list<actasp::AnswerSet> Clingo5_2::filteringQuery(const AnswerSet &currentState, const AnswerSet &plan,
                                                       const std::vector<actasp::AspRule> &goals) {

  //generate a string with all the fluents "0{fluent}1."
  //and add the minimize statement ( eg: :~ pos(x,y,z), ... . [1@1] )
  stringstream fluentsString, minimizeString;

  fluentsString << "#program base." << endl;

  auto fluent = currentState.getFluents().begin();
  for (; fluent != currentState.getFluents().end(); ++fluent) {
    fluentsString << "0{" << fluent->to_string() << "}1." << endl;
    minimizeString << ":~ " << fluent->to_string() << ". [1]" << endl;
  }

  fluentsString << endl;
  minimizeString << endl;

  string monitorString = generateMonitorQuery(goals, plan);

  //combine this plan with all the fluents stuff created before
  stringstream total;
  total << fluentsString.str() << std::endl << monitorString << endl << minimizeString.str() << endl;

  //make a query that only uses
  return makeQuery(goals, plan.getFluents().size(), plan.getFluents().size(), "filterState", 0, false);

}

std::list<actasp::AnswerSet>
Clingo5_2::genericQuery(const std::vector<actasp::AspRule> &query, unsigned int timestep,
                        const std::string &fileName,
                        unsigned int answerSetsNumber) const noexcept {
  return genericQuery(query, timestep, fileName, answerSetsNumber, true);
}


}
