#include <actasp/reasoners/Clingo5_2.h>

#include <actasp/AspRule.h>
#include <actasp/AnswerSet.h>
#include <actasp/AspAtom.h>
#include <actasp/AspProgram.h>
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

Clingo5_2::Clingo5_2(const std::string& incrementalVar,
                     const std::vector<std::string>& linkFiles,
                     const std::vector<std::string>& copyFiles,
                     const std::set<std::string>& actions,
                     unsigned int max_time
                     ) noexcept :
  incrementalVar(incrementalVar),
  max_time(max_time),
  linkFiles(linkFiles),
  copyFiles(copyFiles),
  allActions(actions) {

  for (const auto &linkFile: linkFiles) {
    assert(boost::filesystem::is_regular_file(linkFile));
  }

  for (const auto &copyFile: copyFiles) {
    assert(boost::filesystem::is_regular_file(copyFile));
  }

}


struct RuleToString5_2 {

  std::string operator()(const AspFluentRule& rule) const {

    stringstream ruleStream;

    //iterate over head
    for (int i =0, size = rule.head.size(); i <size; ++i) {

        ruleStream << rule.head[i].to_string();

      if (i < (size-1))
        ruleStream << " | ";
    }

    if (!(rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i =0, size = rule.body.size(); i <size; ++i) {

        ruleStream << rule.body[i].to_string();

      if (i < (size-1))
        ruleStream << ", ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << "." << std::endl;

    return ruleStream.str();
  }

};

struct RuleToCumulativeString5_2 {

  RuleToCumulativeString5_2(Variable timeStepVar) : timeStep(std::move(timeStepVar)) {}

  std::string operator()(const AspFluentRule& rule) const {

    stringstream ruleStream;
    unsigned int headTimeStep = 0;

    //iterate over head
    for (int i =0, size = rule.head.size(); i <size; ++i) {
      ruleStream << with_timestep(rule.head[i], timeStep);
      headTimeStep = std::max(headTimeStep,rule.head[i].getTimeStep());

      if (i < (size-1))
        ruleStream << " | ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i =0, size = rule.body.size(); i <size; ++i) {
      ruleStream << rule.body[i].to_string();

      if (i < (size-1))
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

struct RuleToGoalString5_2 {

  RuleToGoalString5_2(const std::string& timeStepVar) : timeStep(timeStepVar) {}

  std::string operator()(const AspFluentRule& rule) const {

    stringstream ruleStream;
    unsigned int headTimeStep = 0;

    //iterate over head
    for (int i =0, size = rule.head.size(); i <size; ++i) {
        ruleStream << rule.head[i].to_string();
      headTimeStep = std::max(headTimeStep,rule.head[i].getTimeStep());

      if (i < (size-1))
        ruleStream << " | ";
    }

    if (!(rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i =0, size = rule.body.size(); i <size; ++i) {
      ruleStream << with_timestep(rule.body[i], timeStep);

      if (i < (size-1))
        ruleStream << ", ";
    }

    if (!rule.body.empty()) {
        ruleStream << ", ";
        ruleStream << "query(" << timeStep << ")";
    }

    ruleStream << "." << endl;

    return ruleStream.str();
  }

  string timeStep;
};

static AnswerSet model_to_answer_set(const Clingo::Model &model) {
  cout << "Model" << endl;
  vector<AspFluent> set;
  for (auto &atom : model.symbols(Clingo::ShowType::Shown)) {
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
    cout << atom << endl;
  }
  cout << endl << endl;
  return AnswerSet(set.begin(), set.end());
}

static string cumulativeString(const std::vector<actasp::AspFluentRule>& query, const string& timeStepVar) {

  stringstream aspStream;
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToCumulativeString5_2(timeStepVar));
  return aspStream.str();
}

static string aspString(const std::vector<actasp::AspFluentRule>& query, unsigned int timeStep) {
  stringstream aspStream;
  std::vector<actasp::AspFluentRule> rules;
  for (const auto &rule: query) {
    rules.push_back(add_timestep(rule, timeStep));
  }
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToString5_2());
  return aspStream.str();
}

static inline void add(Clingo::Control &control, const actasp::AspProgram &program) {
  std::vector<const char *> as_strings;
  for (const auto &variable: program.getVariables()) {
    as_strings.push_back(variable.name.c_str());
  }
  std::stringstream stream;
  for (const auto &rule: program.getRules()) {
    stream << rule.to_string();
  }

  control.add(program.getName().c_str(), as_strings, stream.str().c_str());
}


std::list<actasp::AnswerSet> Clingo5_2::minimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
    bool filterActions,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const noexcept {


  list<AnswerSet> answers = makeQuery(goalRules,0,max_plan_length,"planQuery",answerset_number);

  if (filterActions)
    return filterPlans(answers,allActions);
  else
    return answers;

}


std::list<actasp::AnswerSet> Clingo5_2::lengthRangePlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
    bool filterActions,
    unsigned int min_plan_length,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const noexcept {


  //cout << "min " << min_plan_length << " max " << max_plan_length << endl;

  std::list<actasp::AnswerSet> allplans =  makeQuery(goalRules,max_plan_length,max_plan_length,"planQuery",answerset_number);

  //clingo 3 generates all plans up to a maximum length anyway, we can't avoid the plans shorter than min_plan_length to be generated
  //we can only filter them out afterwards

  allplans.remove_if(MaxTimeStepLessThan(min_plan_length));

  if (filterActions)
    return filterPlans(allplans,allActions);
  else
    return allplans;

}

actasp::AnswerSet Clingo5_2::optimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
    bool filterActions,
    unsigned int  max_plan_length,
    unsigned int answerset_number,
    bool minimum) const noexcept {

  list<AnswerSet> allAnswers = makeQuery(goalRules, max_plan_length, max_plan_length, "planQuery", answerset_number, true);

  //TODO: Fix this
  AnswerSet optimalPlan = allAnswers.front();
  if (filterActions) {
    list<AnswerSet> sets;
    sets.push_back(optimalPlan);
    return *(filterPlans(sets,allActions).begin());
  }
  else
    return optimalPlan;
}

AnswerSet Clingo5_2::currentStateQuery(const std::vector<actasp::AspFluentRule>& query) const noexcept {
  // TODO: This used to be aspString(query, 0). Fix it
  list<AnswerSet> sets = makeQuery(query,0,0,"stateQuery",1);

  return (sets.empty())? AnswerSet() : *(sets.begin());
}

struct HasTimeStepZeroInHead5_2 : unary_function<const AspFluentRule&,bool> {

  bool operator()(const AspFluentRule &rule) const {
    if (rule.head.empty())
      return false;

    return rule.head[0].getTimeStep() == 0; //I am assuming the heads have a single fluent
    //If the that's not the case, the option --shift has to be added to clingo's command line
  }
};

std::list<actasp::AnswerSet> Clingo5_2::genericQuery(const std::vector<actasp::AspFluentRule>& query,
    unsigned int timeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber, bool useCopyFiles) const noexcept {

  std::vector<actasp::AspFluentRule> base;
  remove_copy_if(query.begin(),query.end(),back_inserter(base), not1(HasTimeStepZeroInHead5_2()));

  string base_part = aspString(base,0);

  stringstream thequery(base_part, ios_base::app | ios_base::out);

  std::vector<actasp::AspFluentRule> cumulative;
  remove_copy_if(query.begin(),query.end(),back_inserter(cumulative), HasTimeStepZeroInHead5_2());

  string cumulative_part = cumulativeString(cumulative,incrementalVar);

  thequery << endl << "#program step(" << incrementalVar << ")." << endl;
  thequery << cumulative_part << endl;

  return makeQuery(query,timeStep,timeStep,fileName,answerSetsNumber);

}

std::string Clingo5_2::generateMonitorQuery(const std::vector<actasp::AspFluentRule>& goalRules,
    const AnswerSet& plan) const noexcept {

  stringstream monitorQuery("", ios_base::app | ios_base::out);

  monitorQuery << "#program step(" << incrementalVar << ")." << endl;


  const AnswerSet::FluentSet &actionSet = plan.getFluents();
  auto actionIt = actionSet.begin();
  vector<AspFluentRule> plan_in_rules;

  for (int i=1; actionIt != actionSet.end(); ++actionIt, ++i) {
    AspFluent action(*actionIt);
    AspFluentRule actionRule;
    actionRule.head.push_back(with_timestep(action, i));
    plan_in_rules.push_back(actionRule);
  }

  monitorQuery << cumulativeString(plan_in_rules,"n");
  
  return monitorQuery.str();
}

std::list<actasp::AnswerSet> Clingo5_2::monitorQuery(const std::vector<actasp::AspFluentRule>& goalRules,
    const AnswerSet& plan) const noexcept {

  //   clock_t kr1_begin = clock();
      
  string monitorQuery = generateMonitorQuery(goalRules,plan);

  list<actasp::AnswerSet> result = makeQuery(goalRules,plan.getFluents().size(),plan.getFluents().size(),"monitorQuery",1);

  result.remove_if(MaxTimeStepLessThan(plan.getFluents().size()));

//   clock_t kr1_end = clock();
//   cout << "Verifying plan time: " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  return result;
}

list<AnswerSet> Clingo5_2::makeQuery(const std::vector<AspFluentRule> &goal, unsigned int initialTimeStep, unsigned int finalTimeStep,
                                 const std::string &fileName, unsigned int answerSetsNumber, bool useCopyFiles) const  noexcept {

  Clingo::Control control;
  using S = std::string;
  using Clingo::Number;
  using Clingo::Function;
  using Clingo::TruthValue;
  using ModelVec = std::vector<Clingo::SymbolVector>;
  using MessageVec = std::vector<std::pair<Clingo::WarningCode, std::string>>;
  auto logger = [](Clingo::WarningCode, char const *message) {
    cerr << message << endl;
  };
  control = Clingo::Control({}, logger, 20);

  control.configuration()["solve"]["models"] = std::to_string(answerSetsNumber).c_str();

  //cout << "initialTimeStep is " << initialTimeStep << " ; finalTimeStep is " << finalTimeStep << endl;
  if (useCopyFiles) {
    for (const auto &path: copyFiles) {
      control.load(path.c_str());
    }
  }
  for (const auto &path: linkFiles) {
    control.load(path.c_str());
  }

  path queryDir = getQueryDirectory(linkFiles, copyFiles);
  auto queryDirFiles = populateDirectory(queryDir, linkFiles, copyFiles);

  const path queryPath = (queryDir / fileName).string() + ".asp";

  AspProgram check("check", {goal}, {Variable("n")});
  //add(control, check);
  // A test goal that should be satisfiable for n=2
  control.add("check", {"n"}, "#external query(n). \n:- not query(2)." );
  std::ofstream query_file(queryPath.string());
  query_file << check.to_string();
  query_file.close();
  //const path outputFilePath = (queryDir / fileName).string() + "_output.txt";
  list<AnswerSet> allAnswers;

  for (int i = 0; i <= 10; ++i) {
    try {
      vector<Clingo::Part> parts;
      parts.emplace_back("check", Clingo::SymbolSpan{Number(i)});
      if (i == 0) {
        parts.emplace_back("base", Clingo::SymbolSpan{});
      } else {
        parts.push_back({"step", {Number(i)}});
        control.release_external(Function("query", {Number(i-1)}));
        //control.cleanup();
      }
      control.ground(parts);
      control.assign_external(Function("query", {Number(2)}), TruthValue::True);


      for (auto atom: control.theory_atoms()) {
        std::cout << atom << std::endl;
      }
      for (auto atom: control.symbolic_atoms()) {
        std::cout << atom.symbol() << std::endl;
      }
      std::cout << "_------ END PROGRAM" << std::endl;

      auto handle = control.solve();
      std::cout << handle.get() << std::endl;
      if (handle.get().is_unsatisfiable()) {
        continue;
      }
      for (const auto &model : handle) {
        allAnswers.push_back(model_to_answer_set(model));
      }
      break;

    }
    catch (exception const &e) {
      cerr << "Grounding failed with: " << e.what() << endl;
    }
  }

  return allAnswers;
}


std::list<actasp::AnswerSet> Clingo5_2::filteringQuery(const AnswerSet& currentState, const AnswerSet& plan,const std::vector<actasp::AspFluentRule>& goals) {

  //generate a string with all the fluents "0{fluent}1."
  //and add the minimize statement ( eg: :~ pos(x,y,z), ... . [1@1] )
  stringstream fluentsString, minimizeString;

  fluentsString << "#program base." << endl;

  auto fluent = currentState.getFluents().begin();
  for (; fluent != currentState.getFluents().end(); ++fluent) {
    fluentsString << "0{" << fluent->to_string() << "}1." << endl;
    minimizeString  << ":~ " << fluent->to_string() << ". [1]" << endl;
  }

  fluentsString << endl;
  minimizeString << endl;

  string monitorString = generateMonitorQuery(goals,plan);

  //combine this plan with all the fluents stuff created before
  stringstream total;
  total << fluentsString.str() << std::endl << monitorString << endl << minimizeString.str() << endl;

  //make a query that only uses
  return  makeQuery(goals,plan.getFluents().size(),plan.getFluents().size(),"filterState",0, false);

}

std::list<actasp::AnswerSet>
Clingo5_2::genericQuery(const std::vector<actasp::AspFluentRule> &query, unsigned int timestep, const std::string &fileName,
                        unsigned int answerSetsNumber) const noexcept {
  return genericQuery(query, timestep, fileName, answerSetsNumber, true);
}



}
