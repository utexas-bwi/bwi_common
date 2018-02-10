#include <actasp/reasoners/Clingo3.h>

#include <actasp/AspRule.h>
#include <actasp/AnswerSet.h>
#include <actasp/AspAtom.h>

#include <algorithm>
#include <iterator>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <iostream>

#define CURRENT_FILE_HOME queryDir // was  std::string("/tmp/")
#define CURRENT_STATE_FILE std::string("current.asp")

using namespace std;

namespace actasp {
  
Clingo3::Clingo3(const std::string& incrementalVar,
         const std::string& queryDir,
         const std::string& domainDir,
         const ActionSet& allActions,
         unsigned int max_time
        ) throw() :
  incrementalVar(incrementalVar),
  max_time(max_time),
  queryDir(queryDir),
  domainDir(domainDir),
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
    setCurrentState(set<AspFluent>());
  
  currentFile.close();

  stringstream filterStream;
  filterStream << "#hide." << endl;

  std::set<AspFluent>::const_iterator actIt = allActions.begin();
  for (; actIt != allActions.end(); ++actIt) {
    filterStream << "#show " << actIt->getName() << "/" << actIt->arity() << "." << endl;
  }

  actionFilter = filterStream.str();
}

struct RuleToString3 {
  RuleToString3(unsigned int timeStepNum) {
    stringstream ss;
    ss << timeStepNum;
    timeStep = ss.str();
  }

  RuleToString3(const std::string& timeStepVar) : timeStep(timeStepVar) {}

  RuleToString3() : timeStep("") {}

  std::string operator()(const AspRule& rule) {

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
  transform(query.begin(),query.end(),ostream_iterator<std::string>(aspStream),RuleToString3(timeStepVar));
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

string Clingo3::generatePlanQuery(std::vector<actasp::AspRule> goalRules,
                                bool filterActions) const throw() {
  stringstream goal;
  goal << "#volatile " << incrementalVar << "." << endl;
  //I don't like this -1 too much, but it makes up for the incremental variable starting at 1
  goal << aspString(goalRules,incrementalVar+"-1") << endl;

  if (filterActions)
    goal << actionFilter;


  return goal.str();
}


std::list<actasp::AnswerSet> Clingo3::minimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const throw() {

  string planquery = generatePlanQuery(goalRules, filterActions);

  return genericQuery(planquery,0,max_plan_length,"planQuery",answerset_number);

}

struct MaxTimeStepLessThan3 {

  MaxTimeStepLessThan3(unsigned int initialTimeStep) : initialTimeStep(initialTimeStep) {}

  bool operator()(const AnswerSet& answer) {
    return !answer.getFluents().empty() &&  answer.maxTimeStep() < initialTimeStep;
  }

  unsigned int initialTimeStep;
};

std::list<actasp::AnswerSet> Clingo3::lengthRangePlanQuery(const std::vector<actasp::AspRule>& goalRules,
    bool filterActions,
    unsigned int min_plan_length,
    unsigned int  max_plan_length,
    unsigned int answerset_number) const throw() {

  string planquery = generatePlanQuery(goalRules, filterActions);

  std::list<actasp::AnswerSet> allplans =  genericQuery(planquery,max_plan_length,max_plan_length,"planQuery",answerset_number);

  //clingo 3 generates all plans up to a maximum length anyway, we can't avoid the plans shorter than min_plan_length to be generated
  //we can only filter them out afterwards

  allplans.remove_if(MaxTimeStepLessThan3(min_plan_length));

  return allplans;

}

AnswerSet Clingo3::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {
  list<AnswerSet> sets = genericQuery(aspString(query,0),0,0,"stateQuery",1);

  return (sets.empty())? AnswerSet() : *(sets.begin());
}

std::list<actasp::AnswerSet> Clingo3::genericQuery(const std::vector<actasp::AspRule>& query,
    unsigned int timeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber) const throw() {
  return genericQuery(aspString(query,""),timeStep,timeStep,fileName,answerSetsNumber);

}

std::list<actasp::AnswerSet> Clingo3::monitorQuery(const std::vector<actasp::AspRule>& goalRules,
    const AnswerSet& plan) const throw() {

  //   clock_t kr1_begin = clock();

  string planQuery = generatePlanQuery(goalRules,true);

  stringstream monitorQuery(planQuery, ios_base::app | ios_base::out);

  const AnswerSet::FluentSet &allActions = plan.getFluents();
  AnswerSet::FluentSet::const_iterator actionIt = allActions.begin();

  for (int i=1; actionIt != allActions.end(); ++actionIt, ++i)
    monitorQuery << actionIt->toString(i) << "." << endl;

  list<actasp::AnswerSet> result = genericQuery(monitorQuery.str(),plan.getFluents().size(),plan.getFluents().size(),"monitorQuery",1);
  
  result.remove_if(MaxTimeStepLessThan3(plan.getFluents().size()));
  
//   clock_t kr1_end = clock();
//   cout << "Verifying plan time: " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  return result;
}

std::string Clingo3::makeQuery(const std::string& query, 
                               unsigned int initialTimeStep, 
                               unsigned int finalTimeStep, 
                               const std::string& fileName,
                              unsigned int answerSetsNumber) const  throw() {
  //this depends on our way of representing stuff.
  //iclingo starts from 1, while we needed the initial state and first action to be at time step 0
  initialTimeStep++;
  finalTimeStep++;

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
  iterations << "--imin=" << initialTimeStep << " --imax=" << finalTimeStep;

  commandLine << "iclingo " << iterations.str() << " " << queryPath << " " << domainDir << "*.asp " << (CURRENT_FILE_HOME + CURRENT_STATE_FILE) << " > " << outputFilePath << " " << answerSetsNumber;


  if (!system(commandLine.str().c_str())) {
    //maybe do something here, or just kill the warning about the return value not being used.
  }
  
  return outputFilePath;
}

std::list<actasp::AnswerSet> Clingo3::genericQuery(const std::string& query,
    unsigned int initialTimeStep,
    unsigned int finalTimeStep,
    const std::string& fileName,
    unsigned int answerSetsNumber) const throw() {

  string outputFilePath = makeQuery(query,initialTimeStep,finalTimeStep,fileName,answerSetsNumber);

  list<AnswerSet> allAnswers = readAnswerSets(outputFilePath);

  return allAnswers;
}

std::list< std::list<AspAtom> > Clingo3::genericQuery(const std::string& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const throw() {
        
string outputFilePath = makeQuery(query,timestep,timestep,fileName,answerSetsNumber);

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

void Clingo3::setCurrentState(const std::set<actasp::AspFluent>& newState) {

  //copy the current state in a file
  ofstream currentFile((CURRENT_FILE_HOME + CURRENT_STATE_FILE).c_str());

  set<AspFluent>::const_iterator stateIt = newState.begin();
  for (; stateIt != newState.end(); ++stateIt)
    currentFile << stateIt->toString(0) << "." << endl;

  currentFile.close();

}


}
