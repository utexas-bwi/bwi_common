#include <actasp/reasoners/Clingo5_2.h>

#include <actasp/AspRule.h>
#include <actasp/AnswerSet.h>
#include <actasp/AspAtom.h>
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

  std::string operator()(const AspRule& rule) const {

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

  std::string operator()(const AspRule& rule) const {

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

  std::string operator()(const AspRule& rule) const {

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


static string cumulativeString(const std::vector<actasp::AspRule>& query, const string& timeStepVar) {

  stringstream aspStream;
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToCumulativeString5_2(timeStepVar));
  return aspStream.str();
}

static string aspString(const std::vector<actasp::AspRule>& query, const Variable& timeStepVar) {


}

static string aspString(const std::vector<actasp::AspRule>& query, unsigned int timeStep) {
  stringstream aspStream;
  std::vector<actasp::AspRule> rules;
  for (const auto &rule: query) {
    rules.push_back(add_timestep(rule, timeStep));
  }
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToString5_2());
  return aspStream.str();
}

static std::list<AspFluent> parseAnswerSet(const std::string& answerSetContent) noexcept {

  stringstream predicateLine(answerSetContent);

  list<AspFluent> predicates;

  //split the line based on spaces
  copy(istream_iterator<string>(predicateLine),
       istream_iterator<string>(),
       back_inserter(predicates));

  return predicates;
}


static actasp::AnswerSet readOptimalAnswerSet(const std::string& filePath, const bool minimum) noexcept {

  ifstream file(filePath.c_str());

  AnswerSet optimalAnswer;
  AnswerSet currentAnswer;
  unsigned int optimization = std::numeric_limits<unsigned int>::max();
  unsigned int currentOptimization;
  bool interrupted = false;

  string line;
  while (file) {

    getline(file,line);

    if(line == "UNSATISFIABLE" || line == "UNKNOWN") {
      return optimalAnswer;
    }

    if (line.find("INTERRUPTED : 1") != string::npos)
      interrupted = true;

    if (line.find("Answer") != string::npos) {
      getline(file,line);
      while (line.find("Answer") != string::npos) 
        getline(file,line);
      try {
        list<AspFluent> fluents = parseAnswerSet(line);
        currentAnswer = AnswerSet(fluents.begin(), fluents.end());
      } catch (std::invalid_argument& arg) {
        //swollow it and skip this answer set.
      }
    }

    if (line.find("Optimization: ") != string::npos) {
      size_t space = line.find_first_of(" ");
      currentOptimization = atoi(line.substr(space+1).c_str());

      if (minimum && (currentOptimization < optimization)) {
        optimalAnswer = currentAnswer;
        optimization = currentOptimization;
      }
      else if ((!minimum) && (currentOptimization > optimization)) {
        optimalAnswer = currentAnswer;
        optimization = currentOptimization;
      }

    }
  }

  return optimalAnswer;
}

string Clingo5_2::generatePlanQuery(std::vector<actasp::AspRule> goalRules) const noexcept {
  stringstream goal;
  goal << "#external query(" << incrementalVar << ")." << endl;
  //I don't like this -1 too much, but it makes up for the incremental variable starting at 1

  transform(goalRules.begin(),goalRules.end(),ostream_iterator<std::string>(goal),RuleToGoalString5_2(incrementalVar));

  goal << endl;

  return goal.str();
}

std::list<actasp::AnswerSet> Clingo5_2::minimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const noexcept {

  string planquery = generatePlanQuery(goalRules);

  list<AnswerSet> answers = genericQuery(planquery,0,max_plan_length,"planQuery",answerset_number);

  if (filterActions)
    return filterPlans(answers,allActions);
  else
    return answers;

}

struct MaxTimeStepLessThan5_2 {

  MaxTimeStepLessThan5_2(unsigned int initialTimeStep) : initialTimeStep(initialTimeStep) {}

  bool operator()(const AnswerSet& answer) {
    return !answer.getFluents().empty() &&  answer.maxTimeStep() < initialTimeStep;
  }

  unsigned int initialTimeStep;
};

std::list<actasp::AnswerSet> Clingo5_2::lengthRangePlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int min_plan_length,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const noexcept {

  string planquery = generatePlanQuery(goalRules);

  //cout << "min " << min_plan_length << " max " << max_plan_length << endl;

  std::list<actasp::AnswerSet> allplans =  genericQuery(planquery,max_plan_length,max_plan_length,"planQuery",answerset_number);

  //clingo 3 generates all plans up to a maximum length anyway, we can't avoid the plans shorter than min_plan_length to be generated
  //we can only filter them out afterwards

  allplans.remove_if(MaxTimeStepLessThan5_2(min_plan_length));

  if (filterActions)
    return filterPlans(allplans,allActions);
  else
    return allplans;

}

actasp::AnswerSet Clingo5_2::optimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int  max_plan_length,
    unsigned int answerset_number,
    bool minimum) const noexcept {

  string planquery = generatePlanQuery(goalRules);

  list<AnswerSet> allAnswers = makeQuery(planquery, max_plan_length, max_plan_length, "planQuery", answerset_number, true);

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

AnswerSet Clingo5_2::currentStateQuery(const std::vector<actasp::AspRule>& query) const noexcept {
  list<AnswerSet> sets = genericQuery(aspString(query,0),0,0,"stateQuery",1);

  return (sets.empty())? AnswerSet() : *(sets.begin());
}

struct HasTimeStepZeroInHead5_2 : unary_function<const AspRule&,bool> {

  bool operator()(const AspRule &rule) const {
    if (rule.head.empty())
      return false;

    return rule.head[0].getTimeStep() == 0; //I am assuming the heads have a single fluent
    //If the that's not the case, the option --shift has to be added to clingo's command line
  }
};

std::list<actasp::AnswerSet> Clingo5_2::genericQuery(const std::vector<actasp::AspRule>& query,
    unsigned int timeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber, bool useCopyFiles) const noexcept {

  std::vector<actasp::AspRule> base;
  remove_copy_if(query.begin(),query.end(),back_inserter(base), not1(HasTimeStepZeroInHead5_2()));

  string base_part = aspString(base,0);

  stringstream thequery(base_part, ios_base::app | ios_base::out);

  std::vector<actasp::AspRule> cumulative;
  remove_copy_if(query.begin(),query.end(),back_inserter(cumulative), HasTimeStepZeroInHead5_2());

  string cumulative_part = cumulativeString(cumulative,incrementalVar);

  thequery << endl << "#program step(" << incrementalVar << ")." << endl;
  thequery << cumulative_part << endl;

  return genericQuery(thequery.str(),timeStep,timeStep,fileName,answerSetsNumber);

}

std::string Clingo5_2::generateMonitorQuery(const std::vector<actasp::AspRule>& goalRules,
    const AnswerSet& plan) const noexcept {
   string planQuery = generatePlanQuery(goalRules);

  stringstream monitorQuery(planQuery, ios_base::app | ios_base::out);

  monitorQuery << "#program step(" << incrementalVar << ")." << endl;


  const AnswerSet::FluentSet &actionSet = plan.getFluents();
  auto actionIt = actionSet.begin();
  vector<AspRule> plan_in_rules;

  for (int i=1; actionIt != actionSet.end(); ++actionIt, ++i) {
    AspFluent action(*actionIt);
    AspRule actionRule;
    actionRule.head.push_back(with_timestep(action, i));
    plan_in_rules.push_back(actionRule);
  }

  monitorQuery << cumulativeString(plan_in_rules,"n");
  
  return monitorQuery.str();
}

std::list<actasp::AnswerSet> Clingo5_2::monitorQuery(const std::vector<actasp::AspRule>& goalRules,
    const AnswerSet& plan) const noexcept {

  //   clock_t kr1_begin = clock();
      
  string monitorQuery = generateMonitorQuery(goalRules,plan);

  list<actasp::AnswerSet> result = genericQuery(monitorQuery,plan.getFluents().size(),plan.getFluents().size(),"monitorQuery",1);

  result.remove_if(MaxTimeStepLessThan5_2(plan.getFluents().size()));

//   clock_t kr1_end = clock();
//   cout << "Verifying plan time: " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  return result;
}

list<AnswerSet> Clingo5_2::makeQuery(const std::string &query, unsigned int initialTimeStep, unsigned int finalTimeStep,
                                 const std::string &fileName, unsigned int answerSetsNumber, bool useCopyFiles) const  noexcept {
  //this depends on our way of representing stuff.
  //iclingo starts from 1, while we needed the initial state and first action to be at time step 0
  initialTimeStep++;
  finalTimeStep++;

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
  control.add("check", {"n"}, query.c_str());

  //const path outputFilePath = (queryDir / fileName).string() + "_output.txt";
  list<AnswerSet> allAnswers;
  try {
    //control.assign_external(Function("query", {Number(0)}), TruthValue::True);
    control.ground({{"step", {Number(1)}}});
    control.ground({{"step", {Number(2)}}});
    control.ground({{"check", {Number(2)}}});

    for (auto atom: control.theory_atoms()) {
      //std::cout << atom << std::endl;
    }
    for (auto atom: control.symbolic_atoms()) {
      //std::cout << atom.symbol() << std::endl;
    }
    auto handle = control.solve();
    for (const auto& m : handle) {
      cout << "Model" << endl;
      vector<AspFluent> set;
      for (auto &atom : m.symbols(Clingo::ShowType::Shown)) {
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
        allAnswers.emplace_back(set.begin(), set.end());
      }
    }
    cout << handle.get() << endl;
  }
  catch (exception const &e) {
    cerr << "Grounding failed with: " << e.what() << endl;
  }

  return allAnswers;
}

std::list<actasp::AnswerSet> Clingo5_2::genericQuery(const std::string& query,
    unsigned int initialTimeStep,
    unsigned int finalTimeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber,
    bool useCopyFiles) const noexcept {

  list<AnswerSet> allAnswers = makeQuery(query, initialTimeStep, finalTimeStep, fileName, answerSetsNumber, useCopyFiles);

  return allAnswers;
}

std::list<actasp::AnswerSet> Clingo5_2::genericQuery(const std::string& query,
    unsigned int timestep,
    const std::string& fileName,
    unsigned int answerSetsNumber, bool useCopyFiles) const noexcept {

  list<AnswerSet> answersets = makeQuery(query, timestep, timestep, fileName, answerSetsNumber, useCopyFiles);

  return answersets;

}

std::list<actasp::AnswerSet> Clingo5_2::filteringQuery(const AnswerSet& currentState, const AnswerSet& plan,const std::vector<actasp::AspRule>& goals) {

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
  return  genericQuery(total.str(),plan.getFluents().size(),plan.getFluents().size(),"filterState",0, false);

}

std::list<actasp::AnswerSet>
Clingo5_2::genericQuery(const std::vector<actasp::AspRule> &query, unsigned int timestep, const std::string &fileName,
                        unsigned int answerSetsNumber) const noexcept {
  return genericQuery(query, timestep, fileName, answerSetsNumber, true);
}

actasp::AnswerSet
Clingo5_2::optimizationQuery(const std::string &query, const std::string &fileName) const noexcept {
  list<AnswerSet> answerSets = makeQuery(query, 0, 0, fileName, 0, true);
  // TODO: fix this
  return answerSets.front();
}


}
