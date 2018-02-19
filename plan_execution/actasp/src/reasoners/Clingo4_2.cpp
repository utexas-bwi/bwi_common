#include <actasp/reasoners/Clingo4_2.h>

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

#define CURRENT_STATE_FILE std::string("/tmp/current.asp")

using namespace std;

namespace actasp {

Clingo4_2::Clingo4_2(const std::string& incrementalVar,
                      const std::string& queryDir,
                      const std::string& domainDir,
                      const ActionSet& allActions,
                      unsigned int max_time
                    ) throw() :
  incrementalVar(incrementalVar),
  max_time(max_time),
  queryDir(queryDir),
  domainDir(domainDir),
  allActions(allActions),
  currentFilePath(CURRENT_STATE_FILE) {

  if (max_time > 0 && !system("timeout 2>/dev/null")) //make sure timeout is available
    max_time = 0;
  
  //make sure directory ends with '/'
  
  if (this->queryDir.find_last_of("/") != (this->queryDir.length() -1))
    this->queryDir += "/";

  if ((this->domainDir.find_last_of("/")) != (this->domainDir.length() -1))
    this->domainDir += "/";

  //TODO test the existance of the directories

  //create current file
  ifstream currentFile(currentFilePath.c_str());
  if (!currentFile.good()) //doesn't exist, create it or clingo will go mad
    setCurrentState(set<AspFluent>());

  currentFile.close();
}

Clingo4_2::Clingo4_2(const std::string& incrementalVar,
                     const std::string& queryDir,
                     const std::string& domainDir,
                     const ActionSet& allActions,
                     const std::string& currentFilePath,
                     unsigned int max_time
                    ) throw() :
  incrementalVar(incrementalVar),
  max_time(max_time),
  queryDir(queryDir),
  domainDir(domainDir),
  allActions(allActions),
  currentFilePath(currentFilePath) {

  if (max_time > 0 && !system("timeout 2>/dev/null")) //make sure timeout is available
    max_time = 0;

  //make sure directory ends with '/'

  if (this->queryDir.find_last_of("/") != (this->queryDir.length() -1))
    this->queryDir += "/";

  if ((this->domainDir.find_last_of("/")) != (this->domainDir.length() -1))
    this->domainDir += "/";

  //TODO test the existance of the directories

  //create current file
  ifstream currentFile((currentFilePath).c_str());
  if (!currentFile.good()) //doesn't exist, create it or clingo will go mad
    setCurrentState(set<AspFluent>());

  currentFile.close();
}

struct RuleToString4_2 {
  RuleToString4_2(unsigned int timeStepNum) {
    stringstream ss;
    ss << timeStepNum;
    timeStep = ss.str();
  }

  RuleToString4_2(const std::string& timeStepVar) : timeStep(timeStepVar) {}

  RuleToString4_2() : timeStep("") {}

  std::string operator()(const AspRule& rule) const {

    stringstream ruleStream;

    //iterate over head
    for (int i =0, size = rule.head.size(); i <size; ++i) {
      if (timeStep.size() >0)
        ruleStream << rule.head[i].toString(timeStep);
      else
        ruleStream << rule.head[i].toString();

      if (i < (size-1))
        ruleStream << ", ";
    }

    if (!(rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i =0, size = rule.body.size(); i <size; ++i) {
      if (timeStep.size() >0)
        ruleStream << rule.body[i].toString(timeStep);
      else
        ruleStream << rule.body[i].toString();

      if (i < (size-1))
        ruleStream << "| ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << "." << std::endl;

    return ruleStream.str();
  }

  string timeStep;
};

struct RuleToCumulativeString4_2 {

  RuleToCumulativeString4_2(const std::string& timeStepVar) : timeStep(timeStepVar) {}

  std::string operator()(const AspRule& rule) const {

    stringstream ruleStream;
    unsigned int headTimeStep = 0;

    //iterate over head
    for (int i =0, size = rule.head.size(); i <size; ++i) {
      ruleStream << rule.head[i].toString(timeStep+"-1");
      headTimeStep = std::max(headTimeStep,rule.head[i].getTimeStep());

      if (i < (size-1))
        ruleStream << " | ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i =0, size = rule.body.size(); i <size; ++i) {
      ruleStream << rule.body[i].toString();

      if (i < (size-1))
        ruleStream << ", ";
    }

    if (!rule.head.empty()) {
      if (!rule.body.empty())
        ruleStream << ", ";

      ruleStream << timeStep << "=" << headTimeStep << "+1";
    }


    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << "." << endl;

    return ruleStream.str();
  }

  string timeStep;
};

struct RuleToGoalString4_2 {

  RuleToGoalString4_2(const std::string& timeStepVar) : timeStep(timeStepVar) {}

  std::string operator()(const AspRule& rule) const {

    stringstream ruleStream;
    unsigned int headTimeStep = 0;

    //iterate over head
    for (int i =0, size = rule.head.size(); i <size; ++i) {
      ruleStream << rule.head[i].toString(timeStep+"-1");
      headTimeStep = std::max(headTimeStep,rule.head[i].getTimeStep());

      if (i < (size-1))
        ruleStream << " | ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i =0, size = rule.body.size(); i <size; ++i) {
      ruleStream << rule.body[i].toString(timeStep+"-1");

      if (i < (size-1))
        ruleStream << ", ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << ", query(" << timeStep << ")." << endl;

    return ruleStream.str();
  }

  string timeStep;
};


static string cumulativeString(const std::vector<actasp::AspRule>& query, const string& timeStepVar) {

  stringstream aspStream;
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToCumulativeString4_2(timeStepVar));
  return aspStream.str();
}

static string aspString(const std::vector<actasp::AspRule>& query, const string& timeStepVar) {

  stringstream aspStream;
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToString4_2(timeStepVar));
  return aspStream.str();
}

static string aspString(const std::vector<actasp::AspRule>& query, unsigned int timeStep) {

  stringstream vs;
  vs << timeStep;

  return aspString(query,vs.str());
}

static std::list<AspFluent> parseAnswerSet(const std::string& answerSetContent) throw() {

  stringstream predicateLine(answerSetContent);

  list<AspFluent> predicates;

  //split the line based on spaces
  copy(istream_iterator<string>(predicateLine),
       istream_iterator<string>(),
       back_inserter(predicates));

  return predicates;
}


static std::list<actasp::AnswerSet> readAnswerSets(const std::string& filePath) throw() {

  ifstream file(filePath.c_str());

  list<AnswerSet> allSets;
  bool interrupted = false;

  string line;
  while (file) {

    getline(file,line);

    if (line == "UNSATISFIABLE")
      return list<AnswerSet>();

    if (line.find("INTERRUPTED : 1") != string::npos)
      interrupted = true;

    if (line.find("Answer") != string::npos) {
      getline(file,line);
      while (line.find("Answer") != string::npos) 
        getline(file,line);
      try {
        list<AspFluent> fluents = parseAnswerSet(line);
        allSets.push_back(AnswerSet(fluents.begin(), fluents.end()));
      } catch (std::invalid_argument& arg) {
        //swollow it and skip this answer set.
      }
    }
  }

  if (interrupted) //the last answer set might be invalid
    allSets.pop_back();

  return allSets;
}

static actasp::AnswerSet readOptimalAnswerSet(const std::string& filePath, const bool minimum) throw() {

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

string Clingo4_2::generatePlanQuery(std::vector<actasp::AspRule> goalRules) const throw() {
  stringstream goal;
  goal << "#program volatile(" << incrementalVar << ")." << endl;
  goal << "#external query(" << incrementalVar << ")." << endl;
  //I don't like this -1 too much, but it makes up for the incremental variable starting at 1

  transform(goalRules.begin(),goalRules.end(),ostream_iterator<std::string>(goal),RuleToGoalString4_2(incrementalVar));

  goal << endl;

  return goal.str();
}


static list<AnswerSet> filterPlans(const list<AnswerSet> unfiltered_plans, const ActionSet& allActions) {

  list<AnswerSet> plans;

  list<AnswerSet>::const_iterator ans = unfiltered_plans.begin();
  for (; ans != unfiltered_plans.end(); ++ans) {
    list<AspFluent> actionsOnly;
    remove_copy_if(ans->getFluents().begin(),ans->getFluents().end(),back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    plans.push_back(AnswerSet(actionsOnly.begin(), actionsOnly.end()));
  }

  return plans;
}

std::list<actasp::AnswerSet> Clingo4_2::minimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const throw() {

  string planquery = generatePlanQuery(goalRules);

  list<AnswerSet> answers = genericQuery(planquery,0,max_plan_length,"planQuery",answerset_number,true);

  if (filterActions)
    return filterPlans(answers,allActions);
  else
    return answers;

}

struct MaxTimeStepLessThan4_2 {

  MaxTimeStepLessThan4_2(unsigned int initialTimeStep) : initialTimeStep(initialTimeStep) {}

  bool operator()(const AnswerSet& answer) {
    return !answer.getFluents().empty() &&  answer.maxTimeStep() < initialTimeStep;
  }

  unsigned int initialTimeStep;
};

std::list<actasp::AnswerSet> Clingo4_2::lengthRangePlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int min_plan_length,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const throw() {

  string planquery = generatePlanQuery(goalRules);

  //cout << "min " << min_plan_length << " max " << max_plan_length << endl;

  std::list<actasp::AnswerSet> allplans =  genericQuery(planquery,max_plan_length,max_plan_length,"planQuery",answerset_number,true);

  //clingo 3 generates all plans up to a maximum length anyway, we can't avoid the plans shorter than min_plan_length to be generated
  //we can only filter them out afterwards

  allplans.remove_if(MaxTimeStepLessThan4_2(min_plan_length));

  if (filterActions)
    return filterPlans(allplans,allActions);
  else
    return allplans;

}

actasp::AnswerSet Clingo4_2::optimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int  max_plan_length,
    unsigned int answerset_number,
    bool minimum) const throw() {

  string planquery = generatePlanQuery(goalRules);

  string outputFilePath = makeQuery(planquery,max_plan_length,max_plan_length,"planQuery",answerset_number,true);

  AnswerSet optimalPlan = readOptimalAnswerSet(outputFilePath,minimum);

  if (filterActions) {
    list<AnswerSet> sets;
    sets.push_back(optimalPlan);
    return *(filterPlans(sets,allActions).begin());
  }
  else
    return optimalPlan;
}

AnswerSet Clingo4_2::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {
  list<AnswerSet> sets = genericQuery(aspString(query,0),0,0,"stateQuery",1,true);

  return (sets.empty())? AnswerSet() : *(sets.begin());
}

struct HasTimeStepZeroInHead4_2 : unary_function<const AspRule&,bool> {

  bool operator()(const AspRule &rule) const {
    if (rule.head.empty())
      return false;

    return rule.head[0].getTimeStep() == 0; //I am assuming the heads have a single fluent
    //If the that's not the case, the option --shift has to be added to clingo's command line
  }
};

std::list<actasp::AnswerSet> Clingo4_2::genericQuery(const std::vector<actasp::AspRule>& query,
    unsigned int timeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber) const throw() {

  std::vector<actasp::AspRule> base;
  remove_copy_if(query.begin(),query.end(),back_inserter(base), not1(HasTimeStepZeroInHead4_2()));

  string base_part = aspString(base,0);

  stringstream thequery(base_part, ios_base::app | ios_base::out);

  std::vector<actasp::AspRule> cumulative;
  remove_copy_if(query.begin(),query.end(),back_inserter(cumulative), HasTimeStepZeroInHead4_2());

  string cumulative_part = cumulativeString(cumulative,incrementalVar);

  thequery << endl << "#program cumulative(" << incrementalVar << ")." << endl;
  thequery << cumulative_part << endl;

  return genericQuery(thequery.str(),timeStep,timeStep,fileName,answerSetsNumber,true);

}

std::string Clingo4_2::generateMonitorQuery(const std::vector<actasp::AspRule>& goalRules,
    const AnswerSet& plan) const throw() {
   string planQuery = generatePlanQuery(goalRules);

  stringstream monitorQuery(planQuery, ios_base::app | ios_base::out);

  monitorQuery << "#program cumulative(" << incrementalVar << ")." << endl;


  const AnswerSet::FluentSet &actionSet = plan.getFluents();
  AnswerSet::FluentSet::const_iterator actionIt = actionSet.begin();
  vector<AspRule> plan_in_rules;

  for (int i=1; actionIt != actionSet.end(); ++actionIt, ++i) {
    AspFluent action(*actionIt);
    action.setTimeStep(i);
    AspRule actionRule;
    actionRule.head.push_back(action);
    plan_in_rules.push_back(actionRule);
  }

  monitorQuery << cumulativeString(plan_in_rules,"n");
  
  return monitorQuery.str();
}

std::list<actasp::AnswerSet> Clingo4_2::monitorQuery(const std::vector<actasp::AspRule>& goalRules,
    const AnswerSet& plan) const throw() {

  //   clock_t kr1_begin = clock();
      
  string monitorQuery = generateMonitorQuery(goalRules,plan);

  list<actasp::AnswerSet> result = genericQuery(monitorQuery,plan.getFluents().size(),plan.getFluents().size(),"monitorQuery",1,true);

  result.remove_if(MaxTimeStepLessThan4_2(plan.getFluents().size()));

//   clock_t kr1_end = clock();
//   cout << "Verifying plan time: " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  return result;
}

std::string Clingo4_2::makeQuery(const std::string& query,
                                 unsigned int initialTimeStep,
                                 unsigned int finalTimeStep,
                                 const std::string& fileName,
                                 unsigned int answerSetsNumber,
                                 bool useCurrentState
                                ) const  throw() {
  //this depends on our way of representing stuff.
  //iclingo starts from 1, while we needed the initial state and first action to be at time step 0
  initialTimeStep++;
  finalTimeStep++;

  //cout << "initialTimeStep is " << initialTimeStep << " ; finalTimeStep is " << finalTimeStep << endl;

  string queryPath = queryDir + fileName + ".asp";

  ofstream queryFile(queryPath.c_str());
  queryFile << query << endl;
  queryFile.close();

  stringstream commandLine;

  const string outputFilePath = queryDir + fileName + "_output.txt";

  if (max_time > 0) {
    commandLine << "timeout " << max_time << " ";
  }

  stringstream iterations;
  iterations << "-cimin=" << initialTimeStep;
//   if ( finalTimeStep > initialTimeStep ) //when max and initial are the same, we do not want max
    iterations << " -cimax=" << finalTimeStep;

  commandLine << "clingo " << iterations.str() << " " << queryPath << " " << domainDir << "*.asp ";
  if(useCurrentState)
    commandLine<< (currentFilePath);
  
  commandLine << " > " << outputFilePath << " " << answerSetsNumber;


  if (!system(commandLine.str().c_str())) {
    //maybe do something here, or just kill the warning about the return value not being used.
  }

  return outputFilePath;
}

std::list<actasp::AnswerSet> Clingo4_2::genericQuery(const std::string& query,
    unsigned int initialTimeStep,
    unsigned int finalTimeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber,
    bool useCurrentState ) const throw() {

  string outputFilePath = makeQuery(query,initialTimeStep,finalTimeStep,fileName,answerSetsNumber,useCurrentState);

  list<AnswerSet> allAnswers = readAnswerSets(outputFilePath);

  return allAnswers;
}

std::list< std::list<AspAtom> > Clingo4_2::genericQuery(const std::string& query,
    unsigned int timestep,
    const std::string& fileName,
    unsigned int answerSetsNumber) const throw() {

  string outputFilePath = makeQuery(query,timestep,timestep,fileName,answerSetsNumber,true);

  ifstream file(outputFilePath.c_str());

  list<list <AspAtom> > allSets;
  bool answerFound = false;

  string line;
  while (file) {

    getline(file,line);

    if (answerFound && line == "UNSATISFIABLE")
      return list<list <AspAtom> >();

    if (line.find("Answer") != string::npos) {
      getline(file,line);
      try {
        stringstream predicateLine(line);

        list<AspAtom> atoms;

        //split the line based on spaces
        copy(istream_iterator<string>(predicateLine),
             istream_iterator<string>(),
             back_inserter(atoms));

        allSets.push_back(atoms);
      } catch (std::invalid_argument& arg) {
        //swollow it and skip this answer set.
      }
    }
  }

  return allSets;

}

void Clingo4_2::setCurrentState(const std::set<actasp::AspFluent>& newState) {

  //copy the current state in a file
  ofstream currentFile((currentFilePath).c_str());

  set<AspFluent>::const_iterator stateIt = newState.begin();
  for (; stateIt != newState.end(); ++stateIt)
    currentFile << stateIt->toString(0) << "." << endl;

  currentFile.close();

}

std::list<actasp::AnswerSet> Clingo4_2::filteringQuery(const AnswerSet& currentState, const AnswerSet& plan,const std::vector<actasp::AspRule>& goals) {

  //generate a string with all the fluents "0{fluent}1."
  //and add the minimize statement ( eg: :~ pos(x,y,z), ... . [1@1] )
  stringstream fluentsString, minimizeString;

  fluentsString << "#program base." << endl;

  std::vector<actasp::AspFluent>::const_iterator fluent = currentState.getFluents().begin();
  for (; fluent != currentState.getFluents().end(); ++fluent) {
    fluentsString << "0{" << fluent->toString() << "}1." << endl;
    minimizeString  << ":~ " << fluent->toString() << ". [1]" << endl;
  }

  fluentsString << endl;
  minimizeString << endl;

  string monitorString = generateMonitorQuery(goals,plan);

  //combine this plan with all the fluents stuff created before
  stringstream total;
  total << fluentsString.str() << std::endl << monitorString << endl << minimizeString.str() << endl;

  //make a query that only uses the domain and what I created, not current.asp
  return  genericQuery(total.str(),plan.getFluents().size(),plan.getFluents().size(),"filterState",0,false);

}


}
