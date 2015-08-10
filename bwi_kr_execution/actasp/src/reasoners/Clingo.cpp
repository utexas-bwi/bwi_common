#include <actasp/reasoners/Clingo.h>

#include "IsNotLocallyOptimal.h"
#include "LexComparator.h"

#include <actasp/AnswerSet.h>
#include <actasp/action_utils.h>
#include <actasp/Action.h>

#include <ctime>

#include <sstream>
#include <ostream>
#include <iterator>
#include <algorithm>
#include <fstream>
#include <cmath> //for floor
#include <boost/graph/graph_concepts.hpp>

//#include <iostream>
#include <ros/console.h>
#include <ctime>

#define CURRENT_FILE_HOME std::string("/tmp/") // was  queryDir, but this doesn't work with the RemoteReasoner at the moment
#define CURRENT_STATE_FILE std::string("current.asp")

using namespace std;

namespace actasp {



//own the actions in the map
Clingo::Clingo(unsigned int max_n,
               const std::string& incrementalVar,
               const std::string& queryDir,
               const std::string& domainDir,
               const ActionSet& actions,
               unsigned int max_time
              ) throw() :
  max_n(max_n),
  incrementalVar(incrementalVar),
  max_time(max_time),
  queryDir(queryDir),
  domainDir(domainDir),
  allActions(actions),
  actionFilter() {

  if (max_time > 0 && !system("timeout 2>/dev/null")) //make sure timeout is available
    max_time = 0;

  //make sure directory ends with '/'

  if (this->queryDir.find_last_of("/") != (this->queryDir.length() -1))
    this->queryDir += "/";

  if ((this->domainDir.find_last_of("/")) != (this->domainDir.length() -1))
    this->domainDir += "/";

  //TODO test the existance of the directories
  
  //create current file
  ifstream currentFile((CURRENT_FILE_HOME + CURRENT_STATE_FILE).c_str());
  if(!currentFile.good()) //doesn't exist, create it or clingo will go mad
    reset();
  currentFile.close();

  stringstream filterStream;
  //filterStream << "#hide." << endl;

  std::set<AspFluent>::const_iterator actIt = allActions.begin();
  for (; actIt != allActions.end(); ++actIt) {
    filterStream << "#show " << actIt->getName() << "/" << actIt->arity() << "." << endl;
  }

  actionFilter = filterStream.str();
}

struct RuleToString {
  RuleToString(unsigned int timeStepNum) {
    stringstream ss;
    ss << timeStepNum;
    timeStep = ss.str();
  }

  RuleToString(const string& timeStepVar) : timeStep(timeStepVar) {}

  std::string operator()(const AspRule& rule) {

    stringstream ruleStream;

    //iterate over head
    for (int i =0, size = rule.head.size(); i <size; ++i) {
      ruleStream << rule.head[i].toString(timeStep);
      if (i < (size-1))
        ruleStream << ", ";
    }

    if (!(rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i =0, size = rule.body.size(); i <size; ++i) {
      ruleStream << rule.body[i].toString(timeStep);
      if (i < (size-1))
        ruleStream << ", ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << "." << std::endl;

    return ruleStream.str();
  }

  string timeStep;
};


static string aspString(const std::vector<actasp::AspRule>& query, const string& timeStepVar) {

  stringstream aspStream;
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToString(timeStepVar));
  return aspStream.str();
}

static string aspString(const std::vector<actasp::AspRule>& query, unsigned int timeStep) {

  stringstream vs;
  vs << timeStep;

  return aspString(query,vs.str());
}

static std::list<AspFluent> parseAnswerSet(const std::string& answerSetContent) {

  stringstream predicateLine(answerSetContent);

  list<AspFluent> predicates;

  //split the line based on spaces
  copy(istream_iterator<string>(predicateLine),
            istream_iterator<string>(),
            back_inserter(predicates));

  return predicates;
}


static std::list<actasp::AnswerSet> readAnswerSets(const std::string& filePath) {

  ifstream file(filePath.c_str());

  list<AnswerSet> allSets;
  bool interrupted = false;
  
  string line;
  while(file) {

    getline(file,line);

    if(line == "UNSATISFIABLE" || line == "UNKNOWN")
      return list<AnswerSet>();
    
    if((line.find("INTERRUPTED") != string::npos) || (line.find("KILLED") != string::npos) )
      interrupted = true;

    if(line.find("Answer") != string::npos) {
      getline(file,line);
        try {
          list<AspFluent> fluents = parseAnswerSet(line);
          allSets.push_back(AnswerSet(fluents.begin(), fluents.end()));
        } catch (std::invalid_argument& arg) {
          //swollow it and skip this answer set.
        }
    }
  }
  
  if(interrupted) //the last answer set might be invalid
    allSets.pop_back();
  
 return allSets;
}

std::list<actasp::AnswerSet> Clingo::krQuery(const std::string& query,
    unsigned int initialTimeStep,
    unsigned int finalTimeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber = 1) const throw() {


  //this depends on our way of representing stuff.
  //iclingo starts from 1, while we needed the initial state and first action to be at time step 0
  initialTimeStep++;
  finalTimeStep++;

  string queryPath = queryDir + fileName;

  ofstream queryFile(queryPath.c_str());
  queryFile << "#program base." << endl;
  queryFile << query << endl;
  queryFile.close();

  stringstream commandLine;

  const string outputFilePath = queryDir + "query_output.txt";

  if (max_time > 0) {
    commandLine << "timeout -k 1 " << max_time << " ";
  }

  stringstream iterations;
  iterations << "-c imin=" << initialTimeStep-1 << " -c iquery=" << initialTimeStep-1 << " -c imax=" << finalTimeStep;

  commandLine << "clingo " << iterations.str() << " " << queryPath << " " << domainDir << "*.asp " << (CURRENT_FILE_HOME + CURRENT_STATE_FILE) << " > " << outputFilePath << " " << answerSetsNumber;

  if (!system(commandLine.str().c_str())) {
    //maybe do something here, or just kill the warning about the return value not being used.
  }

  return readAnswerSets(outputFilePath);

}

string Clingo::generatePlanQuery(std::vector<actasp::AspRule> goalRules,
                                 bool filterActions = true) const throw() {
  stringstream goal;
  goal << "#program volatile(" << incrementalVar << ")." << endl;
  goal << "#external query(" << incrementalVar << ")." << endl;

  std::vector<actasp::AspRule>::iterator ruleIt = goalRules.begin();
  AspFluent query("query(n)");
  for (; ruleIt != goalRules.end(); ++ruleIt) {
    (*ruleIt).body.push_back(query);
  }

  goal << aspString(goalRules,incrementalVar) << endl;

  if (filterActions)
    goal << actionFilter;


  return goal.str();
}



ActionSet Clingo::availableActions() const throw() {
  list<AnswerSet> actions = krQuery(generatePlanQuery(vector<AspRule>(), true),1,1,"planQuery.asp",0);

  ActionSet computed;

  //the first answer set contains the current state and no actions
  list<AnswerSet>::iterator act = ++actions.begin();
  for (; act != actions.end(); ++act) {
    computed.insert(act->getFluents().begin(), act->getFluents().end());
  }

  return computed;
}


AnswerSet Clingo::computePlan(const std::vector<actasp::AspRule>& goal) const throw() {

  list<AnswerSet> answerSets;

  string query = generatePlanQuery(goal);

  answerSets = krQuery(query,0,max_n,"planQuery.asp");


  if (answerSets.empty())
    return AnswerSet();

  return  *(answerSets.begin());
}

struct PolicyMerger {

  PolicyMerger(MultiPolicy & policy) :
    policy(policy) {}

  void operator()(const AnswerSet& set) {
    policy.merge(set);
  }

  MultiPolicy &policy;
};

//substituted with the check on sub-sequences

// static set<AspFluent> reducePlan(const list<AspFluent> &plan, int begin, int length) {
//
//   set<AspFluent> newPlan;
//
//   list<AspFluent>::const_iterator actionIt = plan.begin();
//
//   for (int timeStep = 0; actionIt != plan.end(); ++timeStep, ++actionIt) {
//     if (timeStep < begin) {
//       newPlan.insert(*actionIt);
//     } else if (timeStep >= begin + length) {
//       AspFluent fluentCopy(*actionIt);
//       fluentCopy.setTimeStep(timeStep - length);
//       newPlan.insert(fluentCopy);
//     }
//   }
//
//   return newPlan;
// }
//
// struct IsNotLocallyOptimal {
//
//   IsNotLocallyOptimal(const Clingo *reasoner,
//                       const std::vector<actasp::AspRule>& goal,
//                       const ActionSet &allActions ) : reasoner(reasoner), goal(goal), allActions(allActions) {}
//
//   bool operator()(const AnswerSet& plan) const {
//
//     list<AspFluent> actionsOnly;
//
//     remove_copy_if(plan.getFluents().begin(), plan.getFluents().end(),
//                      back_inserter(actionsOnly),not1(IsAnAction(allActions)));
//
//     for (int length = 1; length < actionsOnly.size(); ++length) {
//
//       for (int begin = 0; begin <= actionsOnly.size() - length; ++begin) {
//
//         //remove length actions from begin
//         set<AspFluent> reduced = reducePlan(actionsOnly,begin,length);
//
//         if (reasoner->isPlanValid(AnswerSet(true,reduced),goal))
//           return true;
//
//       }
//     }
//
//     return false;
//   }
//
//   const Clingo *reasoner;
//   const vector<actasp::AspRule>& goal;
//   const ActionSet &allActions;
// };


//checks wheter the shorter plan occurs in the longer one. For instance: ABCDEF and ABCDXYEF removing XY

struct IsSubSequence {

  IsSubSequence(const list< AspFluent> &myPlan) : longerPlan(myPlan) {};

  bool operator()(const list<AspFluent> &shorterPlan) const {

    pair< list< AspFluent>::const_iterator, list< AspFluent>::const_iterator>
    start = mismatch(shorterPlan.begin(),shorterPlan.end(),longerPlan.begin(), ActionEquality());

    size_t remaining = distance(start.first, shorterPlan.end());
    list< AspFluent>::const_iterator secondStart = longerPlan.end();
    advance(secondStart, -remaining);

    return equal(start.first, shorterPlan.end(), secondStart, ActionEquality());

  }

  const list< AspFluent> &longerPlan;

};

struct IsNotLocallyOptimalSubPlanCheck {

  IsNotLocallyOptimalSubPlanCheck(const set< list< AspFluent>,LexComparator > &allPlans, const ActionSet &allActions) :
    allPlans(allPlans), allActions(allActions) {}

  bool operator()(const AnswerSet& plan) const {
    list<AspFluent> actionsOnly;

    remove_copy_if(plan.getFluents().begin(), plan.getFluents().end(),
                   back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    set< list< AspFluent>, LexComparator >::const_iterator sub = find_if(allPlans.begin(),allPlans.end(),IsSubSequence(actionsOnly));

    return sub != allPlans.end();
  }

  const set< list< AspFluent>, LexComparator > &allPlans;
  const ActionSet &allActions;
};

struct CleanPlan {

  CleanPlan(const ActionSet &allActions) : allActions(allActions) {}

  list<AspFluentRef> operator()(const AnswerSet &planWithStates) const {
    list<AspFluentRef> actionsOnly;

    remove_copy_if(planWithStates.getFluents().begin(), planWithStates.getFluents().end(),
                   back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    return actionsOnly;

  }

  const ActionSet &allActions;

};

struct PlanLongerThan {

  PlanLongerThan(unsigned int length) : length(length) {}

  bool operator()(const AnswerSet& plan) const {
    return plan.maxTimeStep() > length;
  }

  unsigned int length;
};

struct AnswerSetRef {
  AnswerSetRef(const AnswerSet& aset) : aset(&aset) {}

  operator const AnswerSet&() const {
    return *aset;
  }

  const AnswerSet *aset;
};



MultiPolicy Clingo::computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) {

  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }

  string query = generatePlanQuery(goal,false);

  clock_t kr1_begin = clock();
  list<AnswerSet> firstAnswerSets = krQuery(query,1,max_n,"planQuery.asp",0);
  clock_t kr1_end = clock();
//   cout << "The first kr call took " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  MultiPolicy policy(allActions);

  if (firstAnswerSets.empty())
    return policy;

  unsigned int shortestLength = firstAnswerSets.begin()->maxTimeStep();

  for_each(firstAnswerSets.begin(),firstAnswerSets.end(),PolicyMerger(policy));

  int maxLength = ceil(suboptimality * shortestLength);

  if (maxLength == shortestLength)
    return policy;

  //all accepted plans sorted lexicographically
  set< list <AspFluentRef>, LexComparator > goodPlans;

  //remove the states from the plans
  transform(firstAnswerSets.begin(),firstAnswerSets.end(),inserter(goodPlans,goodPlans.begin()), CleanPlan(allActions));

  query = generatePlanQuery(goal,false);

  clock_t kr2_begin = clock();
  list<AnswerSet> answerSets = krQuery(query,maxLength,maxLength,"planQuery.asp",0);
  clock_t kr2_end = clock();
//   cout << "The second kr call took " << (double(kr2_end - kr2_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  //skip the minimial plans
  list<AnswerSet>::iterator currentFirst = find_if(answerSets.begin(),answerSets.end(),PlanLongerThan(answerSets.begin()->maxTimeStep()));

  set< list <AspFluentRef>, LexComparator > badPlans;
  //this object remembers the set of bad plans, cannot be created inside the loop
  IsNotLocallyOptimal isNotLocallyOptimal(&goodPlans,&badPlans, allActions, shortestLength,false);

  clock_t filter_begin = clock();
  while (currentFirst != answerSets.end()) {

    //process the plans in groups of increasing length

    list<AnswerSet>::iterator currentLast = find_if(currentFirst,answerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()));

    list<AnswerSetRef> goodPointers;
    remove_copy_if(currentFirst,currentLast,back_inserter(goodPointers),isNotLocallyOptimal);

    for_each(goodPointers.begin(),goodPointers.end(),PolicyMerger(policy));

    transform(goodPointers.begin(),goodPointers.end(),inserter(goodPlans, goodPlans.begin()), CleanPlan(allActions));

    currentFirst = currentLast;

  }
  clock_t filter_end = clock();


  stringstream planStream;
  planStream << "Accepted plans: " << endl;
  set< list <AspFluentRef>, LexComparator >::const_iterator printIt = goodPlans.begin();
  for (; printIt != goodPlans.end(); ++printIt) {
    copy(printIt->begin(),printIt->end(),ostream_iterator<string>(planStream, " "));
    planStream << endl;
  }
  ROS_INFO_STREAM(planStream.str());
//   
//   cout << "filtering took " << (double(filter_end - filter_begin) / CLOCKS_PER_SEC) << " seconds" << endl;


  return policy;

}

struct AnswerSetToList {
  list <AspFluentRef> operator()(const AnswerSet& aset) const {

    return list <AspFluentRef>(aset.getFluents().begin(), aset.getFluents().end());

  }
};

struct ListToAnswerSet {
  AnswerSet operator()(const list<AspFluent>& plan) {
    return AnswerSet(plan.begin(), plan.end());
  }

};


std::vector< AnswerSet > Clingo::computeAllPlans(const std::vector<actasp::AspRule>& goal,
    double suboptimality) const throw () {

  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }

  string query = generatePlanQuery(goal,true);
  list<AnswerSet> firstAnswerSets = krQuery(query,0,max_n,"planQuery.asp",0);

  if (firstAnswerSets.empty())
    return vector<AnswerSet>();

  //when actions are filtered and there are not state fluents,
  //the last time step is of the last action, and actions start at
  //zero, so we need +1
  //^not true for clingo 4
  unsigned int shortestLength = firstAnswerSets.begin()->maxTimeStep();

  int maxLength = ceil(suboptimality * shortestLength);

  if (maxLength == shortestLength)
    return vector<AnswerSet>(firstAnswerSets.begin(), firstAnswerSets.end());

  set< list <AspFluentRef>, LexComparator > goodPlans;
  transform(firstAnswerSets.begin(), firstAnswerSets.end(),inserter(goodPlans,goodPlans.begin()),AnswerSetToList());

  query = generatePlanQuery(goal,true);
  list<AnswerSet> moreAnswerSets = krQuery(query,maxLength,maxLength,"planQuery.asp",0);


  list<AnswerSet>::iterator currentFirst = find_if(moreAnswerSets.begin(),moreAnswerSets.end(),PlanLongerThan(moreAnswerSets.begin()->maxTimeStep()));

  set< list<AspFluentRef>, LexComparator > badPlans;
  //this object remembers the set of bad plans, cannot be created inside the loop

  IsNotLocallyOptimal isNotLocallyOptimal(&goodPlans, &badPlans,allActions,shortestLength,true);

  list<AnswerSetRef> goodPointers(firstAnswerSets.begin(),firstAnswerSets.end());
  while (currentFirst != moreAnswerSets.end()) {

    //process the plans in groups of increasing length

    list<AnswerSet>::iterator currentLast = find_if(currentFirst,moreAnswerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()));

    size_t size_pre_copy = goodPointers.size();

    remove_copy_if(currentFirst,currentLast,back_inserter(goodPointers),isNotLocallyOptimal);

    list<AnswerSetRef>::iterator from = goodPointers.begin();
    advance(from,size_pre_copy);
    transform(from, goodPointers.end(),inserter(goodPlans,goodPlans.begin()),AnswerSetToList());

    currentFirst = currentLast;
  }

  vector<AnswerSet> finalVector(goodPointers.begin(),goodPointers.end());
  
//   cout << "  ---  good plans ---" << endl;
//   vector< AnswerSet>::const_iterator printIt = finalVector.begin();
//   for (; printIt != finalVector.end(); ++printIt) {
//     copy(printIt->getFluents().begin(),printIt->getFluents().end(),ostream_iterator<string>(cout, " "));
//     cout << endl;
//   }
//   cout << " ---- " << endl;

  return finalVector;


}


bool Clingo::isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw() {

//   clock_t kr1_begin = clock();

  string planQuery = generatePlanQuery(goal);

  stringstream monitorQuery(planQuery, ios_base::app | ios_base::out);

  monitorQuery << "#program plan(" << incrementalVar << ")." << endl;

  const AnswerSet::FluentSet &allActions = plan.getFluents();
  AnswerSet::FluentSet::const_iterator actionIt = allActions.begin();

  for (int i=1; actionIt != allActions.end(); ++actionIt, ++i)
    monitorQuery << actionIt->toString(i) << "." << endl;

  bool valid = krQuery(monitorQuery.str(),plan.getFluents().size(),plan.getFluents().size(),"monitorQuery.asp").empty();
//   clock_t kr1_end = clock();
//   cout << "Verifying plan time: " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  return !valid; 
}

AnswerSet Clingo::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {
  //ROS_INFO("current state query.\n");

  list<AnswerSet> sets = krQuery(aspString(query,0),0,0,"stateQuery.asp");

  if (sets.empty()) {
    return AnswerSet();
  } else {
    std::set<actasp::AspFluent> currentFluents = (sets.begin())->getFluentsAtTime(0);
    return AnswerSet(currentFluents.begin(), currentFluents.end());
  }


  //return (sets.empty())? AnswerSet() : *(sets.begin());
}

static AspRule fluent2Rule(const AspFluent& fluent) {
  AspRule rule;
  rule.head.push_back(fluent);
  return rule;
}

bool Clingo::updateFluents(const std::vector<actasp::AspFluent> &observations) throw() {

  //copy the observations in the heads of rules
  vector<AspRule> obsRules;
  obsRules.reserve(observations.size());
  transform(observations.begin(),observations.end(),back_inserter(obsRules),fluent2Rule);
  //add the rule for the noop action

  stringstream queryStream(aspString(obsRules,1), ios_base::app | ios_base::out);

  queryStream << "noop(1)." << endl;
  //queryStream << "#hide noop/1." << endl;

  list<AnswerSet> currentState = krQuery(queryStream.str(),1,1,"observationQuery.asp");

  if (currentState.empty())
    return false; //the observations are incompatible with the current state and are discarded

  vector<AspRule> newStateRules;
  //iclingo generates all answer sets up to the last iteration, so we need the last one here
  set<AspFluent> newStateFluents = currentState.rbegin()->getFluentsAtTime(1);
  newStateRules.reserve(newStateFluents.size());
  transform(newStateFluents.begin(),newStateFluents.end(),back_inserter(newStateRules),fluent2Rule);

  //copy the current state in a file
  ofstream currentFile((CURRENT_FILE_HOME + CURRENT_STATE_FILE).c_str());

  currentFile << aspString(newStateRules,0);
  currentFile.close();

  return true;
}


//this is almost brutally copied from krquery
std::list< std::list<AspAtom> > Clingo::query(const std::string &queryString, unsigned int initialTimeStep,
                                   unsigned int finalTimeStep) const throw() {

  //this depends on our way of representing stuff.
  //iclingo starts from 1, while we needed the initial state and first action to be at time step 0
  initialTimeStep++;
  finalTimeStep++;

  string queryPath = queryDir + "generic_query.asp";

  ofstream queryFile(queryPath.c_str());
  queryFile << queryString << endl;
  queryFile.close();

  stringstream commandLine;

  const string outputFilePath = queryDir + "query_output.txt";

  if (max_time > 0) {
    commandLine << "timeout -k 1 " << max_time << " ";
  }

  stringstream iterations;
  iterations << "-c imin=" << initialTimeStep-1 << " -c iquery=" << initialTimeStep-1 << " -c imax=" << finalTimeStep;


  commandLine << "clingo " << iterations.str() << " " << domainDir <<  "*.asp " << " " << (CURRENT_FILE_HOME + CURRENT_STATE_FILE) << " " << queryPath <<  " > " << outputFilePath << " 0";


  if (!system(commandLine.str().c_str())) {
    //maybe do something here, or just kill the warning about the return value not being used.
  }

  ifstream file(outputFilePath.c_str());

  list<list <AspAtom> > allSets;
  bool answerFound = false;

  string line;
  while(file) {

    getline(file,line);

    if(answerFound && line == "UNSATISFIABLE")
      return list<list <AspAtom> >();

    if(line.find("Answer") != string::npos) {
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

void Clingo::reset() throw() {
  ofstream current((CURRENT_FILE_HOME + CURRENT_STATE_FILE).c_str());
  current << "";
  current.close();
}

}
